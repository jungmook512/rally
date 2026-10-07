"""rally_robot/dsr_helpers.py — 두산 DSR 초기화와 좌표 변환 공통 유틸 (선생님 robot_control.py에서 분리)
담당: 정정인

사용법 (반드시 rclpy.init() 이후, DSR_ROBOT2 import 전에):
    from rally_robot.dsr_helpers import init_dsr, transform_to_base
    dsr_node = init_dsr()                     # DR_init 설정 + dsr 노드 생성
    from DSR_ROBOT2 import movel, movej, ... # 이제 import 가능
"""
import os
import time
import warnings
from contextlib import contextmanager

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from ament_index_python.packages import get_package_share_directory

warnings.filterwarnings("ignore", message="Gimbal lock detected")   # ry≈180 에서 ZYZ 분해 경고. 값은 유효하다.

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
JHOME_POS = [0, -30, 90, 0, 90, 0]
JREADY_POS = [0, 0, 90, 0, 90, 0]          # 선생님 코드의 init_robot 자세
OBSERVE_POS = JREADY_POS                   # TODO(정정인): 테이블 전체가 보이는 관측 자세로 교체 (skills.yaml)

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -35.0                       # 카메라 깊이 → 파지 높이 보정 [mm]
MIN_DEPTH = 2.0

PACKAGE_PATH = get_package_share_directory("rally_robot")
GRIPPER2CAM_PATH = os.path.join(PACKAGE_PATH, "resource", "T_gripper2camera.npy")


def init_dsr(node_name="rally_dsr_node"):
    """DR_init 전역 설정 후 dsr 네임스페이스 노드를 만들어 돌려준다. 프로세스당 한 번만.
    DR_init 은 여기서만 import 한다 — 이 모듈 자체는 두산 패키지 없이도 import 되어야 한다 (--dry-run, PC2/PC3 테스트)."""
    import DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node(node_name, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    return node


def move_stop(node, stop_mode=2, timeout_s=1.0):
    """진행 중인 모션 정지. DSR_ROBOT2 에 래퍼가 없어 /dsr01/dsr_controller2/motion/move_stop 서비스를 직접 부른다.
    stop_mode: 0 QSTOP_STO · 1 QSTOP · 2 SSTOP(소프트, 기본) · 3 HOLD. 성공 여부 반환."""
    from dsr_msgs2.srv import MoveStop
    import DSR_ROBOT2
    prefix = getattr(DSR_ROBOT2, "_srv_name_prefix", "dsr_controller2/")   # dsr_controller2 는 자기 이름을 서비스 앞에 붙인다
    client = node.create_client(MoveStop, f"/{ROBOT_ID}/{prefix}motion/move_stop")
    if not client.wait_for_service(timeout_sec=timeout_s):
        return False
    req = MoveStop.Request()
    req.stop_mode = int(stop_mode)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    return bool(future.done() and future.result() is not None and future.result().success)


def get_robot_pose_matrix(x, y, z, rx, ry, rz):
    """posx (mm, ZYZ deg) → 4x4 동차변환 (base → gripper)."""
    R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def load_gripper2cam(path=GRIPPER2CAM_PATH):
    return np.load(path)


def transform_to_base(camera_coords, robot_posx, gripper2cam=None):
    """카메라 좌표 (x,y,z mm) → 로봇 베이스 좌표. eye-in-hand: base2cam = base2gripper @ gripper2cam."""
    if gripper2cam is None:
        gripper2cam = load_gripper2cam()
    coord = np.append(np.array(camera_coords, dtype=float), 1.0)
    base2gripper = get_robot_pose_matrix(*robot_posx)
    base2cam = base2gripper @ gripper2cam
    return (base2cam @ coord)[:3]


def posx_to_pose(posx):
    """DRL posx [x,y,z mm, rx,ry,rz deg(ZYZ)] → geometry_msgs/Pose (m, quaternion). 메시지에 실을 때는 항상 이걸 거친다."""
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = posx[0] / 1000.0, posx[1] / 1000.0, posx[2] / 1000.0
    qx, qy, qz, qw = Rotation.from_euler("ZYZ", posx[3:6], degrees=True).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = float(qx), float(qy), float(qz), float(qw)
    return pose


def pose_to_posx(position_m, quat_xyzw):
    """geometry_msgs 좌표 (m, quaternion) → DRL posx (mm, ZYZ deg). DRL 호출 직전에만 쓴다."""
    rx, ry, rz = Rotation.from_quat(list(quat_xyzw)).as_euler("ZYZ", degrees=True)
    return [position_m[0] * 1000.0, position_m[1] * 1000.0, position_m[2] * 1000.0, float(rx), float(ry), float(rz)]


def apply_depth_offset(td_coord):
    td = list(td_coord)
    if td[2] and sum(td) != 0:
        td[2] += DEPTH_OFFSET
        td[2] = max(td[2], MIN_DEPTH)
    return td


# ---------------------------------------------------------------- 스킬 공용: 안전 체크가 들어간 이동 · 정지 확인 · 순응 제어 컨텍스트
STOP_MODES = (2, 1)          # 정지 재시도 순서: SSTOP(소프트) → QSTOP(퀵)
STOP_VERIFY_S = 1.5          # 정지 명령 후 check_motion()==0 을 기다리는 시간 (시도당)
START_GRACE_S = 3.0          # amovel 직후 움직임이 시작되기까지 기다리는 최대 시간. 실로봇: 직전 이동 직후의 amovel 은 ~1 s 늦게 출발한다
START_MOVE_MM = 1.0          # 시작 판정: check_motion != 0 이거나 자세가 이만큼 변하면 '출발'
START_MOVE_DEG = 0.5
ARRIVE_TOL_MM = 3.0          # 도착 검증: 목표와 위치 차이 허용
ARRIVE_TOL_DEG = 2.0         # 도착 검증: 자세 차이 허용
SETTLE_STABLE_S = 0.4        # check_motion 이 0 이 된 뒤 자세가 이 시간 동안 안 변하면 '멈춤'으로 본다


class SkillAbort(Exception):
    """안전 사유로 스킬 중단. stopped=False 면 정지 명령이 실패해 로봇이 아직 움직일 수 있다."""

    def __init__(self, reason: str, stopped: bool = True):
        super().__init__(reason)
        self.reason, self.stopped = reason, stopped


def force_mag(f6):
    return float(np.linalg.norm(f6[:3]))


def rotate_tool(pose6, axis_xyz, deg):
    """posx(mm, ZYZ deg) 의 자세를 툴 좌표계 축(axis_xyz) 기준으로 deg 만큼 돌린 posx 를 돌려준다. 위치는 그대로."""
    r = Rotation.from_euler("ZYZ", pose6[3:6], degrees=True) * Rotation.from_rotvec(np.deg2rad(deg) * np.array(axis_xyz, dtype=float))
    rx, ry, rz = r.as_euler("ZYZ", degrees=True)
    return [float(pose6[0]), float(pose6[1]), float(pose6[2]), float(rx), float(ry), float(rz)]


def stop_robot(dsr_node, logger, why="safety"):
    """정지 서비스 호출 → check_motion()==0 으로 실제 정지 확인. 실패하면 정지 모드를 올려 재시도. 멈춘 것이 확인되면 True."""
    import DSR_ROBOT2
    from DSR_ROBOT2 import check_motion
    for attempt, mode in enumerate(STOP_MODES, 1):
        if dsr_node is not None:
            ok = move_stop(dsr_node, stop_mode=mode)
        else:                                                   # fake 모드: 가짜 모듈의 정지 흉내
            ok = bool(getattr(DSR_ROBOT2, "fake_move_stop", lambda m=2: True)(mode))
        t0 = time.monotonic()
        while time.monotonic() - t0 < STOP_VERIFY_S:
            if check_motion() == 0:
                logger.warning(f"정지 확인 ({why}) mode={mode} 시도 {attempt} 서비스 응답={ok}")
                return True
            time.sleep(0.05)
        logger.error(f"정지 확인 실패 ({why}) mode={mode} 시도 {attempt} 서비스 응답={ok} — 아직 움직임, 재시도")
    logger.fatal(f"정지 실패 ({why}) — 로봇이 아직 움직일 수 있음. 티치펜던트/하드웨어 비상정지 사용")
    return False


def motion_timeout_s(from6, to6, vel_mm_s, floor_s=10.0):
    """거리/속도 ×2 + 5 s. movel 의 vel 은 mm/s 라 60×0.3=18 mm/s 같은 저속이면 수십 초가 정상이다 — 고정 20 s 는 오탐."""
    dist = float(np.linalg.norm(np.array(to6[:3], dtype=float) - np.array(from6[:3], dtype=float)))
    return max(floor_s, dist / max(float(vel_mm_s), 1.0) * 2.0 + 5.0)


def pose_error(cur6, target6):
    """(위치 오차 mm, 자세 오차 deg) — ZYZ 표현 차이가 아니라 실제 회전각으로 비교."""
    dpos = float(np.linalg.norm(np.array(cur6[:3], dtype=float) - np.array(target6[:3], dtype=float)))
    d = Rotation.from_euler("ZYZ", cur6[3:6], degrees=True).inv() * Rotation.from_euler("ZYZ", target6[3:6], degrees=True)
    return dpos, float(np.degrees(np.linalg.norm(d.as_rotvec())))


def move_checked(node, pose6, vel, acc, timeout_s=None, poll_s=0.02, on_poll=None, via=None, stall_ok=False):
    """amovel(via 가 있으면 amovec: 현재 → via → pose6 원호) + 폴링.
    stall_ok=True: 목표 전에 멈춰 선 것을 실패가 아니라 'stalled' 로 돌려준다 — 순응 하강에서 물체·테이블에 닿으면
      스프링처럼 버티며 멈추고(강성 400 N/m × 6 mm ≈ 2.5 N), 힘 임계에 못 미칠 수 있다 (10/7 실로봇).
    원호 도중 감속 배율이 바뀌면 중간부터 같은 원호를 다시 그릴 수 없으므로 정지 확인 후 SkillAbort('speed_changed_during_arc'). node 는 stop_requested()/speed_scale()/safety_reason()/dsr/get_logger() 를 제공해야 한다.
    - 출발 판정: check_motion != 0 **또는** 자세가 1 mm·0.5° 이상 변함 (실로봇은 check_motion 이 늦게 BUSY 로 바뀐다). 3 s 안에 출발 없으면
      정지 명령 후 SkillAbort('not_started')
    - 종료 판정: check_motion == 0 이고 자세가 0.4 s 동안 안정. 그 뒤 목표와 3 mm·2° 안이어야 'done', 아니면 정지 명령 후 SkillAbort('not_arrived')
    - STOP → 정지 확인 후 SkillAbort(stopped=확인 여부). 감속 배율 변화 → 정지 확인 후 같은 자세로 재명령
    - on_poll() 이 True 를 돌려주면(예: 접촉 감지) 정지하고 'interrupted'
    - timeout_s 가 None 이면 현재 자세 → 목표 거리와 속도로 계산(motion_timeout_s)"""
    from DSR_ROBOT2 import amovel, check_motion, get_current_posx, posx
    if via is not None:
        from DSR_ROBOT2 import amovec
    logger = node.get_logger()
    while True:
        if node.stop_requested():
            raise SkillAbort(node.safety_reason(), stopped=True)
        scale = node.speed_scale()
        slowed = scale < 1.0
        start_pose = get_current_posx()[0]
        d0, a0 = pose_error(start_pose, pose6)
        if d0 <= ARRIVE_TOL_MM and a0 <= ARRIVE_TOL_DEG:
            return "done"                                                 # 이미 목표 자세 — 컨트롤러가 움직이지 않으므로 '출발 없음' 오판 방지
        # vel·acc 는 숫자(직선 mm/s) 또는 [직선 mm/s, 회전 deg/s] — 자세가 크게 도는 이동(스윙)은 회전 속도를 같이 줘야 빠르다
        sc = (lambda v: [float(x) * scale for x in v]) if isinstance(vel, (list, tuple)) else (lambda v: float(v) * scale)
        v_cmd, a_cmd = sc(vel), (sc(acc) if isinstance(acc, (list, tuple)) else float(acc) * scale)
        v_lin = v_cmd[0] if isinstance(v_cmd, list) else v_cmd
        if timeout_s is None:
            timeout_s = (motion_timeout_s(start_pose, via, v_lin) + motion_timeout_s(via, pose6, v_lin)) if via is not None \
                else motion_timeout_s(start_pose, pose6, v_lin)
        if via is not None:
            ret = amovec(posx(*via), posx(*pose6), vel=v_cmd, acc=a_cmd)
        else:
            ret = amovel(posx(*pose6), vel=v_cmd, acc=a_cmd)
        if ret not in (None, 0):
            raise SkillAbort(f"amovel_rejected_{ret}", stopped=True)      # 컨트롤러가 명령 거부 (도달 불가·모드 등)
        t0 = time.monotonic()
        started, stable_since, last = False, None, None
        while True:
            time.sleep(poll_s)
            now = time.monotonic()
            if node.stop_requested():
                raise SkillAbort(node.safety_reason(), stopped=stop_robot(node.dsr, logger, "safety STOP"))
            if on_poll is not None and on_poll():
                if not stop_robot(node.dsr, logger, "조건 충족(접촉)"):
                    raise SkillAbort("stop_failed_on_contact", stopped=False)
                return "interrupted"
            if (node.speed_scale() < 1.0) != slowed:
                if not stop_robot(node.dsr, logger, "감속 배율 변경"):
                    raise SkillAbort("stop_failed_on_slow", stopped=False)
                if via is not None:
                    raise SkillAbort("speed_changed_during_arc", stopped=True)
                logger.info(f"안전 레벨 변경(speed_override={node.speed_scale():.2f}) — 같은 자세로 재이동")
                break                                                     # 바깥 while 이 STOP 재확인 후 재명령
            busy = check_motion() != 0
            cur = get_current_posx()[0]
            dpos0, dang0 = pose_error(cur, start_pose)
            if busy or dpos0 > START_MOVE_MM or dang0 > START_MOVE_DEG:
                started = True
            if not started:
                if now - t0 > START_GRACE_S:
                    stopped = stop_robot(node.dsr, logger, "출발 없음")      # 늦게 출발할 모션이 있으면 취소
                    logger.error(f"{START_GRACE_S:.0f} s 안에 움직임 없음 — 컨트롤러가 명령을 무시/지연. 목표와 {pose_error(cur, pose6)[0]:.1f} mm")
                    raise SkillAbort("not_started", stopped=stopped)
                continue
            if now - t0 > timeout_s:
                stopped = stop_robot(node.dsr, logger, "이동 타임아웃")
                raise SkillAbort(f"motion_timeout_{timeout_s:.0f}s", stopped=stopped)
            # 정지 판정: 기준 자세에서 0.4 s 동안 0.3 mm·0.2° 안에 머물러야 정지. (예전엔 한 주기 0.2 mm 기준이라
            # 10 mm/s 미만의 느린 하강을 정지로 착각했다 — 10/7 실로봇 공 놓기)
            if busy or last is None or pose_error(cur, last)[0] > 0.3 or pose_error(cur, last)[1] > 0.2:
                last, stable_since = cur, now
                continue
            if now - stable_since < SETTLE_STABLE_S:
                continue
            dpos, dang = pose_error(cur, pose6)                           # 멈춤 → 도착 검증
            if (dpos > ARRIVE_TOL_MM or dang > ARRIVE_TOL_DEG) and stall_ok:
                if not stop_robot(node.dsr, logger, "접촉으로 멈춤"):          # 남은 명령 취소
                    raise SkillAbort("stop_failed_on_stall", stopped=False)
                logger.info(f"목표 {dpos:.1f} mm 앞에서 멈춤 — 접촉으로 판단")
                return "stalled"
            if dpos > ARRIVE_TOL_MM or dang > ARRIVE_TOL_DEG:
                stopped = stop_robot(node.dsr, logger, "목표 미도달")
                logger.error(f"목표 미도달: 위치 {dpos:.1f} mm · 자세 {dang:.1f}° 차이 — 명령이 거부되었거나 중단됨")
                raise SkillAbort("not_arrived", stopped=stopped)
            return "done"


def movej_checked(node, posj6, vel, acc, timeout_s=30.0, poll_s=0.02):
    """amovej + 폴링 (관절 이동판 move_checked). 관측 자세 복귀처럼 팔 형태를 바꿔야 하는 이동에 쓴다.
    STOP → stop_robot → SkillAbort. 출발 못 하면(이미 그 자세가 아닌데 3 s 동안 check_motion 0) 'not_started'.
    반환 "done". posj6: deg."""
    from DSR_ROBOT2 import amovej, check_motion, get_current_posj
    logger = node.get_logger()
    target = [float(v) for v in posj6]
    if max(abs(a - b) for a, b in zip([float(v) for v in get_current_posj()], target)) < 0.5:
        return "done"
    if node.stop_requested():
        raise SkillAbort(node.safety_reason(), stopped=True)
    scale = node.speed_scale()
    ret = amovej(target, vel=vel * scale, acc=acc * scale)
    if ret is not None and ret < 0:
        raise SkillAbort(f"amovej_failed({ret})", stopped=True)
    t0 = time.monotonic()
    started = False
    while True:
        time.sleep(poll_s)
        if node.stop_requested():
            raise SkillAbort(node.safety_reason(), stopped=stop_robot(node.dsr, logger, "safety STOP"))
        moving = check_motion() != 0
        started = started or moving
        if not started and time.monotonic() - t0 > START_GRACE_S:
            if max(abs(a - b) for a, b in zip([float(v) for v in get_current_posj()], target)) < 0.5:
                return "done"
            raise SkillAbort("not_started", stopped=stop_robot(node.dsr, logger, "출발 없음"))
        if started and not moving:
            return "done"
        if time.monotonic() - t0 > timeout_s:
            raise SkillAbort("timeout", stopped=stop_robot(node.dsr, logger, "관절 이동 시간 초과"))


def controller_alive(node, timeout_s=2.0):
    """두산 motion 서비스(check_motion)가 timeout 안에 응답하는지. 컨트롤러 노드가 반쯤 멈추면(aux 는 답하고 motion 은 무응답) 여기서 걸린다.
    node 는 rclpy 노드(스핀 중이 아닌 것). 응답 없으면 False."""
    import DSR_ROBOT2
    if hasattr(DSR_ROBOT2, "fake_move_stop"):                 # fake 모드: 컨트롤러 없음 → 항상 살아 있음
        return True
    from dsr_msgs2.srv import CheckMotion
    prefix = getattr(DSR_ROBOT2, "_srv_name_prefix", "dsr_controller2/")
    client = node.create_client(CheckMotion, f"/{ROBOT_ID}/{prefix}motion/check_motion")
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False
        future = client.call_async(CheckMotion.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
        return future.done() and future.result() is not None
    finally:
        node.destroy_client(client)


COMPLIANCE_SETTLE_S = 0.3   # 10/7 실로봇: task_compliance_ctrl 직후 곧바로 amovel 을 보내면 알람 없이 무시된다 (0.1 s 만 기다려도 받음)


@contextmanager
def compliance(stx, logger=None):
    """task_compliance_ctrl(stx) 를 켜고, 블록을 어떻게 빠져나가든(예외·취소 포함) 반드시 해제한다.
    켠 뒤 COMPLIANCE_SETTLE_S 기다린다 — 바로 이동 명령을 보내면 컨트롤러가 버린다. 힘 영점도 이 뒤에 읽을 것."""
    from DSR_ROBOT2 import task_compliance_ctrl, release_compliance_ctrl
    task_compliance_ctrl(list(stx))
    time.sleep(COMPLIANCE_SETTLE_S)
    try:
        yield
    finally:
        try:
            release_compliance_ctrl()
        except Exception as e:  # 해제 실패는 로그만 — 호출자의 예외를 덮지 않는다
            if logger:
                logger.error(f"release_compliance_ctrl 실패: {e}")
