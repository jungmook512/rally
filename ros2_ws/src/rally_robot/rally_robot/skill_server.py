"""rally_robot/skill_server.py — RALLY 로봇 스킬 action 서버 7종 (PushBall · PickCompliant · Handover · ResistPush · CatchBall · DetachBall · MoveTarget)
담당: 정정인

설계 메모
- 선생님 코드는 import 시점에 rclpy.init()+DR_init 을 하고 movel/mwait 가 블로킹이다.
  여기서는 main()에서 init_dsr() 한 번 호출하고, 스킬 콜백은 ReentrantCallbackGroup 에서 돌려
  safety_monitor 구독(별도 그룹)이 막히지 않게 한다. Executor 는 MultiThreadedExecutor.
- 모든 스킬은 실행 중 SafetyStatus.level == STOP 이면 즉시 abort 하고 순응 해제 → 후퇴 (dsr_helpers.move_checked / compliance).
  안전 상태를 한 번도 못 받았거나 status_timeout_s 넘게 끊겨도 STOP 으로 본다 (#14) — safety_monitor 없이 띄우면 goal 거부.
- DRL 함수(movel, task_compliance_ctrl, ...)는 init_dsr() 이후 import.
- 접촉이 예정된 구간(집기 하강·핸드오버 당김 대기·저항 밀기)에서는 /rally/safety/contact_expected 를 True 로 올린다(5 Hz 하트비트).
  safety_monitor 는 이 동안 nudge(힘 급변) 판정만 끄고 절대 힘 한계는 그대로 둔다. skill_server 가 죽으면 0.5 s 뒤 자동으로 꺼진다.
- 환자가 물체·손잡이를 잡는 구간(건네기 당김 대기 → 후퇴, 저항 밀기)에서만 /rally/safety/patient_contact 도 올린다(5 Hz).
  safety_monitor 는 이 동안 손 거리 STOP 을 SLOW 로 낮춘다(#59). **이 플래그를 켠 동안 스킬은 환자 쪽(+x)으로 명령 이동을 하지 않는다**
  — 제자리 순응 대기와 환자에게서 멀어지는 이동만. 치기·집기·놓기·ReturnBall 에서는 켜지 않는다.
- 설정: rally_bringup/config/skills.yaml + table.yaml (share 에 설치된 것 → 없으면 소스 트리 폴백).
- 로봇 없이: `ros2 run rally_robot skill_server --fake` 또는 RALLY_FAKE=1 → rally_robot.fake_dsr 를 DSR_ROBOT2 자리에 끼운다. 실제 모드 경로는 그대로.
"""
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from rally_interfaces.action import CatchBall, DetachBall, Handover, MoveTarget, PickCompliant, PushBall, ResistPush, ReturnBall
from rally_interfaces.msg import HandState, SafetyStatus, TrackedObjectArray
from rally_interfaces.srv import GripperCommand
from rally_robot.dsr_helpers import OBSERVE_POS, ROBOT_ID, SkillAbort, init_dsr, motion_speeds

CONTACT_HZ = 5.0
CONFIG_FILES = ("skills.yaml", "table.yaml")


def load_config(logger):
    """skills.yaml + table.yaml 을 한 dict 로. share 에 설치된 파일 → 소스 트리(ros2_ws/src/rally_bringup/config) 순."""
    cfg = {}
    here = os.path.dirname(os.path.abspath(__file__))
    for name in CONFIG_FILES:
        candidates = []
        try:
            candidates.append(os.path.join(get_package_share_directory("rally_bringup"), "config", name))
        except PackageNotFoundError:
            pass
        candidates.append(os.path.normpath(os.path.join(here, "..", "..", "rally_bringup", "config", name)))
        path = next((c for c in candidates if os.path.exists(c)), None)
        if path is None:
            logger.warning(f"{name} 을 찾지 못함 — 스킬 기본값 사용")
            continue
        with open(path, encoding="utf-8") as f:
            cfg.update(yaml.safe_load(f) or {})
        logger.info(f"config 로드: {path}")
    return cfg


class SkillServer(Node):
    def __init__(self, fake=False):
        super().__init__("rally_skill_server")
        self.fake = fake
        self.dsr = None                      # main() 에서 init_dsr() 노드를 넣어준다 (fake 면 None)
        self.cfg = load_config(self.get_logger())
        self.cb_safety = MutuallyExclusiveCallbackGroup()
        self.cb_state = MutuallyExclusiveCallbackGroup()
        self.cb_skill = ReentrantCallbackGroup()
        self.safety = SafetyStatus()
        self.safety_rx = None                # 마지막 /rally/safety/status 수신 monotonic. None = 한 번도 못 받음 → STOP 취급 (#14)
        self.declare_parameter("status_timeout_s", 0.5)   # safety.yaml rally_skill_server. 이보다 오래 못 받으면 STOP 취급
        self.last_hand = None
        self._contact = False
        self._patient_contact = False
        self._cm_client = None
        self.last_pick = None                # PickCompliant 성공 자세 {pose(mm·deg), grip_force} — 모드 5 내려놓기 자리
        self.holding = False                 # 물체를 쥐고 있는지 — gripper() 가 닫힘+감지면 True, 열면 False
        self.depth = None                    # (monotonic 수신 시각, uint16 mm 배열) — 모드 5 받기 대기 중에만 구독 (watch_depth)
        self.cam_K = None                    # 컬러 camera_info K (정렬 깊이와 같은 화면)
        self._depth_subs = []
        self.cb_depth = MutuallyExclusiveCallbackGroup()   # 깊이 영상(1280x720 30 Hz)이 손·그리퍼 상태 콜백을 막지 않게 따로

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable1 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(SafetyStatus, "/rally/safety/status", self._on_safety, reliable, callback_group=self.cb_safety)
        self.create_subscription(HandState, "/rally/perception/hand", self._on_hand, qos_profile_sensor_data, callback_group=self.cb_state)
        self.gripper_state = None            # (monotonic 수신 시각, 폭 mm, grip 비트) — gripper_node 의 실제 상태
        self.create_subscription(JointState, "/rally/gripper/state", self._on_gripper_state, qos_profile_sensor_data, callback_group=self.cb_state)
        self.objects = None                  # (monotonic 수신 시각, TrackedObjectArray) — 비전 물체 (모드 1 공 집기)
        self.create_subscription(TrackedObjectArray, "/rally/perception/objects", self._on_objects, qos_profile_sensor_data, callback_group=self.cb_state)
        self.image_axis = None               # 비전 물체 화면 주축 각도 (objects 와 같은 순서) — 모드 5 받기 기울기
        self.create_subscription(PoseArray, "/rally/perception/objects/image_axis", self._on_image_axis, qos_profile_sensor_data,
                                 callback_group=self.cb_state)
        self.pub_contact = self.create_publisher(Bool, "/rally/safety/contact_expected", reliable1)
        self.pub_patient_contact = self.create_publisher(Bool, "/rally/safety/patient_contact", reliable1)
        self.create_timer(1.0 / CONTACT_HZ, self._contact_tick, callback_group=self.cb_state)
        self.gripper_cli = self.create_client(GripperCommand, "/rally/gripper/command", callback_group=self.cb_state)
        self.create_service(Trigger, "/rally/robot/go_observe_pose", self._go_observe, callback_group=self.cb_skill)
        self.create_service(Trigger, "/rally/robot/go_hand_view", self._go_hand_view, callback_group=self.cb_skill)
        self.create_service(Trigger, "/rally/robot/go_pick_view", self._go_pick_view, callback_group=self.cb_skill)

        from rally_robot.skills import (catch_ball, detach_ball, handover, move_target, pick_compliant, push_ball, resist_push,
                                        return_ball)
        self._servers = [
            ActionServer(self, PushBall, "/rally/skill/push_ball", push_ball.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, CatchBall, "/rally/skill/catch_ball", catch_ball.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, MoveTarget, "/rally/skill/move_target", move_target.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, DetachBall, "/rally/skill/detach_ball", detach_ball.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, Handover, "/rally/skill/handover", handover.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, ResistPush, "/rally/skill/resist_push", resist_push.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, PickCompliant, "/rally/skill/pick_compliant", pick_compliant.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
            ActionServer(self, ReturnBall, "/rally/skill/return_ball", return_ball.execute(self), callback_group=self.cb_skill,
                         goal_callback=self._accept, cancel_callback=self._cancel),
        ]
        self.get_logger().info(f"skill_server ready ({'FAKE' if fake else 'real'} DSR)")

    # ----- 안전 -----
    def _on_safety(self, msg):
        self.safety = msg
        self.safety_rx = time.monotonic()

    def safety_stale(self) -> bool:
        """안전 상태를 한 번도 못 받았거나 status_timeout_s 넘게 끊김 (#14) — safety_monitor 가 없거나 죽음."""
        return self.safety_rx is None or time.monotonic() - self.safety_rx > float(self.get_parameter("status_timeout_s").value)

    def _on_objects(self, msg):
        self.objects = (time.monotonic(), msg)

    def _on_image_axis(self, msg):
        self.image_axis = msg

    def _on_gripper_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        if "rg2_width_mm" in d:
            self.gripper_state = (time.monotonic(), float(d["rg2_width_mm"]), int(round(d.get("rg2_grip", 0.0))),
                                  float(d.get("rg2_target_mm", -1.0)))

    def holding_now(self, min_width_mm: float = 20.0, max_age_s: float = 1.0) -> bool:
        """물체를 쥐고 있는지. 스킬 서버가 보낸 명령 기록(self.holding) 또는 그리퍼 실제 상태(grip 비트, 또는 명령 폭보다 벌어져 멈춤)가 쥠이면 True.
        그리퍼 서비스를 손으로 불러 공을 쥐게 한 경우도 잡는다."""
        if self.holding:
            return True
        st = self.gripper_state
        if st is None or time.monotonic() - st[0] > max_age_s:
            return False
        width, grip, target = st[1], st[2], st[3]
        if width < min_width_mm:          # 빈손으로 닫으면 폭 12.4 mm 에 grip 비트 1 (10/7) — 폭으로 먼저 거른다
            return False
        # gripper_node 의 object_detected 와 같은 규칙: grip 비트 또는 '닫으라 했는데 5 mm 이상 벌어져 멈춤'
        # (10/7 실로봇: 공을 손가락 끝에 걸쳐 쥐면 grip 비트가 0 → 빈손으로 오판해 공을 쥔 채 백스윙했다)
        return grip == 1 or (0.0 <= target < width - 5.0)

    def _on_hand(self, msg):
        self.last_hand = msg

    def watch_depth(self, on: bool):
        """손목 RealSense 정렬 깊이·camera_info 구독 켜기/끄기 — 큰 영상이라 받기 대기 동안만 (handover RECEIVE)."""
        if on and not self._depth_subs:
            self.depth = None
            self._depth_subs = [
                self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", self._on_depth, qos_profile_sensor_data,
                                         callback_group=self.cb_depth),
                self.create_subscription(CameraInfo, "/camera/color/camera_info", self._on_cam_info, qos_profile_sensor_data,
                                         callback_group=self.cb_depth)]
        elif not on and self._depth_subs:
            for sub in self._depth_subs:
                self.destroy_subscription(sub)
            self._depth_subs, self.depth = [], None

    def _on_depth(self, msg):
        import numpy as np
        if msg.encoding not in ("16UC1", "mono16"):
            return
        self.depth = (time.monotonic(), np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width])

    def _on_cam_info(self, msg):
        self.cam_K = [list(msg.k[0:3]), list(msg.k[3:6]), list(msg.k[6:9])]

    def stop_requested(self):
        """STOP 이거나 안전 상태가 끊겼으면 참 → 새 goal 거부, 실행 중 스킬은 check_stop 에서 abort (#14)."""
        return self.safety_stale() or self.safety.level == SafetyStatus.LEVEL_STOP

    def safety_reason(self):
        if self.safety_stale():
            return "safety_status_stale"
        return self.safety.reason or "safety_stop"

    def speed_scale(self):
        return self.safety.speed_override if self.safety.level == SafetyStatus.LEVEL_SLOW else 1.0

    def check_stop(self, goal_handle=None):
        """단계 사이에 호출. STOP 이면 SkillAbort, 취소 요청이면 SkillAbort('canceled')."""
        if self.stop_requested():
            raise SkillAbort(self.safety_reason(), stopped=True)
        if goal_handle is not None and goal_handle.is_cancel_requested:
            raise SkillAbort("canceled", stopped=True)

    def check_controller(self, timeout_s=2.0):
        """두산 motion 서비스가 응답하는지 (fake 면 통과). 무응답이면 SkillAbort('controller_unresponsive') — bringup 재시작 안내."""
        if self.fake:
            return
        import DSR_ROBOT2
        from dsr_msgs2.srv import CheckMotion
        prefix = getattr(DSR_ROBOT2, "_srv_name_prefix", "dsr_controller2/")
        if self._cm_client is None:
            self._cm_client = self.create_client(CheckMotion, f"/{ROBOT_ID}/{prefix}motion/check_motion", callback_group=self.cb_state)
        if not self._cm_client.wait_for_service(timeout_sec=timeout_s):
            raise SkillAbort("controller_unresponsive", stopped=True)
        fut = self._cm_client.call_async(CheckMotion.Request())
        t0 = time.monotonic()
        while not fut.done():
            time.sleep(0.02)
            if time.monotonic() - t0 > timeout_s:
                self.get_logger().error("두산 컨트롤러 motion 서비스 무응답 — bringup 재시작 필요")
                raise SkillAbort("controller_unresponsive", stopped=True)

    # ----- 접촉 예정 플래그 -----
    def set_contact(self, on: bool):
        self._contact = bool(on)
        self.pub_contact.publish(Bool(data=self._contact))

    def set_patient_contact(self, on: bool):
        """환자가 물체·손잡이를 잡는 구간 (#59). 켠 동안 환자 쪽(+x) 명령 이동 금지 — 위 설계 메모."""
        self._patient_contact = bool(on)
        self.pub_patient_contact.publish(Bool(data=self._patient_contact))

    def _contact_tick(self):
        if self._contact:
            self.pub_contact.publish(Bool(data=True))
        if self._patient_contact:
            self.pub_patient_contact.publish(Bool(data=True))

    # ----- 그리퍼 -----
    def gripper_cfg(self, object_class: str):
        """skills.yaml gripper.<class> → (열림 폭 mm, 파지 힘 N). 없으면 기본 (110, 20)."""
        d = (self.cfg.get("gripper") or {}).get(object_class) or {}
        return float(d.get("open_mm", 110.0)), float(d.get("force_n", 20.0))

    def gripper(self, width_mm: float, force_n: float, wait: bool = True, timeout_s: float = 4.0):
        """/rally/gripper/command 호출 → (success, actual_width_mm, object_detected). fake 면 닫을 때 물체 있음으로 흉내."""
        ok, width, detected = self._gripper_call(width_mm, force_n, wait, timeout_s)
        if ok:
            self.holding = bool(detected) if width_mm < 50.0 else False
        return ok, width, detected

    def _gripper_call(self, width_mm, force_n, wait, timeout_s):
        if self.fake:
            # 가짜 그리퍼: 물체를 놓은 자리를 기억 — 그 자리에서 3 cm 넘게 떨어진 곳에서 닫으면 빈손(폭 0).
            # 처음 집기(놓은 적 없음)·놓은 자리 근처에서 닫기는 물체 있음(폭 45). 치기 전 '빈손 확인'을 흉내 내기 위함.
            time.sleep(0.3)
            from DSR_ROBOT2 import get_current_posx
            here = get_current_posx()[0][:3]
            rel = getattr(self, "_fake_release_xyz", None)
            if width_mm >= 50.0:
                if self.holding:
                    self._fake_release_xyz = list(here)
                return True, float(width_mm), False
            near = rel is None or sum((a - b) ** 2 for a, b in zip(here, rel)) ** 0.5 < 30.0
            if near:
                self._fake_release_xyz = None
            return True, (45.0 if near else 0.0), near
        if not self.gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/rally/gripper/command 서비스 없음 — gripper_node 를 먼저 띄우세요")
            return False, 0.0, False
        req = GripperCommand.Request()
        req.width_mm, req.force_n, req.wait_until_done = float(width_mm), float(force_n), bool(wait)
        fut = self.gripper_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            time.sleep(0.02)
            if time.monotonic() - t0 > timeout_s:
                self.get_logger().error("그리퍼 서비스 응답 타임아웃")
                return False, 0.0, False
        r = fut.result()
        return bool(r.success), float(r.actual_width_mm), bool(r.object_detected)

    # ----- action plumbing -----
    def _accept(self, goal):
        return GoalResponse.REJECT if self.stop_requested() else GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def observe_x_max_mm(self) -> float:
        """관측 자세로 가는 관절 경로의 TCP x 한계 mm — skills.yaml observe_pose.route_x_max_m (기본 0.54, 보라색 선 0.57 − 3 cm)."""
        return float((self.cfg.get("observe_pose") or {}).get("route_x_max_m", 0.54)) * 1000.0

    def _go_observe(self, req, res):
        """관측 자세 (관절 이동). 10/11: 동기 movej+mwait 는 이동 내내 두산 서비스(get_tool_force)를 막아 힘이 끊기고
        wrench_lost STOP — 다른 이동과 같이 amovej + 폴링(movej_checked, STOP 이면 멈춤)으로."""
        from rally_robot.dsr_helpers import movej_safe
        if self.stop_requested():
            res.success, res.message = False, f"safety STOP ({self.safety_reason()})"
            return res
        posj = (self.cfg.get("observe_pose") or {}).get("posj") or OBSERVE_POS   # skills.yaml 티칭 값 우선
        tv, ta, _, _, jv, ja = motion_speeds(self)                                 # skills.yaml motion: joint_vel_deg_s
        try:
            movej_safe(self, [float(v) for v in posj], jv, ja, tv, ta, self.observe_x_max_mm())
        except SkillAbort as e:
            res.success, res.message = False, e.reason
            return res
        res.success, res.message = True, f"observe pose {posj}"
        return res


    def _go_pick_view(self, req, res):
        """모드 5 물체 보기 자세 — 관측 자세(관절 이동) → mode5.pick_view_posx 로 직선 이동.
        10/11: 관측 자세에선 흰 배경(물체 놓는 자리, −y 쪽)이 카메라 화면 아래로 잘림. 관측 자세를 거쳐 가서 IK 해가 튀지 않게."""
        from rally_robot.dsr_helpers import move_checked, movej_safe
        if self.stop_requested():
            res.success, res.message = False, f"safety STOP ({self.safety_reason()})"
            return res
        pose = (self.cfg.get("mode5") or {}).get("pick_view_posx")
        posj = (self.cfg.get("observe_pose") or {}).get("posj") or OBSERVE_POS
        tv, ta, _, _, jv, ja = motion_speeds(self, "mode5")
        try:
            movej_safe(self, [float(v) for v in posj], jv, ja, tv, ta, self.observe_x_max_mm())
            if pose:
                move_checked(self, [float(v) for v in pose], tv, ta)
        except SkillAbort as e:
            res.success, res.message = False, e.reason
            return res
        res.success, res.message = True, f"pick view {pose or posj}"
        return res

    def _go_hand_view(self, req, res):
        """손 보기 자세 (모드 1 시도 전 레인 고르기 — 툴을 환자 쪽으로 기울여 RealSense 가 환자 손을 봄). 관절 이동."""
        from rally_robot.dsr_helpers import movej_to_pose
        from rally_robot.skills.push_ball import DEFAULTS as PUSH_DEFAULTS
        from rally_robot.skills.push_ball import hand_view_joint, hand_view_pose
        if self.stop_requested():
            res.success, res.message = False, f"safety STOP ({self.safety_reason()})"
            return res
        m1 = self.cfg.get("mode1") or {}
        cfg = {**PUSH_DEFAULTS, **{k: v for k, v in m1.items() if k in PUSH_DEFAULTS}}
        pose = hand_view_pose(cfg, m1.get("lanes") or [], self.cfg.get("table") or {})
        if pose is None:
            res.success, res.message = False, "no_lanes (skills.yaml mode1.lanes)"
            return res
        try:
            movej_to_pose(self, pose, *hand_view_joint(self, cfg))
        except SkillAbort as e:
            res.success, res.message = False, e.reason
            return res
        res.success, res.message = True, f"hand view {[round(v, 1) for v in pose]}"
        return res


def _install_sim_force(node):
    """시뮬 전용(RALLY_SIM=1, 에뮬레이터): 에뮬레이터에는 외력이 없으니 /rally/sim/external_force(Vector3, N, base)를
    get_tool_force 에 더한다. scripts/sim_gripper.py 의 환자 흉내(당기기·밀어 넣기)가 발행. 실로봇 경로에는 안 들어간다."""
    import DSR_ROBOT2
    from geometry_msgs.msg import Vector3
    ext = [0.0, 0.0, 0.0]

    def on_force(m):
        ext[:] = [m.x, m.y, m.z]

    node.create_subscription(Vector3, "/rally/sim/external_force", on_force, 10)
    orig = DSR_ROBOT2.get_tool_force

    def get_tool_force(*a, **k):
        f = list(orig(*a, **k))
        return [f[0] + ext[0], f[1] + ext[1], f[2] + ext[2], *f[3:]]

    DSR_ROBOT2.get_tool_force = get_tool_force
    node.get_logger().warning("시뮬 모드: /rally/sim/external_force 를 툴 힘에 더함 (RALLY_SIM=1)")


def main(args=None):
    argv = list(sys.argv[1:] if args is None else args)
    fake = "--fake" in argv or os.environ.get("RALLY_FAKE") == "1"
    rclpy.init(args=[a for a in argv if a != "--fake"])
    dsr_node = None
    if fake:
        from rally_robot import fake_dsr
        sys.modules["DSR_ROBOT2"] = fake_dsr        # 스킬의 `from DSR_ROBOT2 import …` 가 가짜를 받는다
    else:
        dsr_node = init_dsr()                        # DR_init 먼저
    node = SkillServer(fake=fake)
    node.dsr = dsr_node
    if not fake and os.environ.get("RALLY_SIM") == "1":
        _install_sim_force(node)
        m1 = node.cfg.get("mode1") or {}
        if not m1.get("rack_drop_posx") and m1.get("sim_rack_drop_posx"):   # 시뮬 전용 거치대 자리 (직접교시 전)
            m1["rack_drop_posx"] = m1["sim_rack_drop_posx"]
            node.get_logger().warning(f"시뮬 모드: rack_drop_posx ← sim_rack_drop_posx {m1['rack_drop_posx']}")
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    if dsr_node is not None:
        ex.add_node(dsr_node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        if dsr_node is not None:
            dsr_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
