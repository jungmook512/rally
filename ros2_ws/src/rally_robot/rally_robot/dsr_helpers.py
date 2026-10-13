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
NEAR_DONE_MM = 8.0           # 다시 보낸 명령이 출발하지 않아도 목표와 이만큼 안이면 도착으로 본다 (10/12)
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


def rotate_base(pose6, axis_xyz, deg):
    """posx 의 자세를 베이스 좌표계 축(axis_xyz) 기준으로 deg 만큼 돌린 posx. 위치는 그대로.
    예: 환자 쪽(+x) 축으로 돌리면 쥔 물체가 환자 기준 좌우로 기울어진다 (모드 5 손목 회전)."""
    r = Rotation.from_rotvec(np.deg2rad(deg) * np.array(axis_xyz, dtype=float)) * Rotation.from_euler("ZYZ", pose6[3:6], degrees=True)
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


def move_checked(node, pose6, vel, acc, timeout_s=None, poll_s=0.02, on_poll=None, via=None, stall_ok=False, path=None, retarget=None):
    """amovel(via 가 있으면 amovec: 현재 → via → pose6 원호) + 폴링.
    path=[posx, ...]: 경유점을 지나 pose6 까지 멈추지 않고 이어 가는 스플라인(amovesx) — 그리퍼를 쓰지 않는 구간을 부드럽게.
      도중 감속 배율이 바뀌면 원호처럼 정지 확인 후 SkillAbort('speed_changed_during_arc').
    stall_ok=True: 목표 전에 멈춰 선 것을 실패가 아니라 'stalled' 로 돌려준다 — 순응 하강에서 물체·테이블에 닿으면
      스프링처럼 버티며 멈추고(강성 400 N/m × 6 mm ≈ 2.5 N), 힘 임계에 못 미칠 수 있다 (10/7 실로봇).
    원호 도중 감속 배율이 바뀌면 중간부터 같은 원호를 다시 그릴 수 없으므로 정지 확인 후 SkillAbort('speed_changed_during_arc'). node 는 stop_requested()/speed_scale()/safety_reason()/dsr/get_logger() 를 제공해야 한다.
    - 출발 판정: check_motion != 0 **또는** 자세가 1 mm·0.5° 이상 변함 (실로봇은 check_motion 이 늦게 BUSY 로 바뀐다). 3 s 안에 출발 없으면
      정지 명령 후 SkillAbort('not_started')
    - 종료 판정: check_motion == 0 이고 자세가 0.4 s 동안 안정. 그 뒤 목표와 3 mm·2° 안이어야 'done', 아니면 정지 명령 후 SkillAbort('not_arrived')
    - STOP → 정지 확인 후 SkillAbort(stopped=확인 여부). 감속 배율 변화 → 정지 확인 후 같은 자세로 재명령
    - on_poll() 이 True 를 돌려주면(예: 접촉 감지) 정지하고 'interrupted'
    - retarget() 이 새 posx 를 돌려주면 **멈추지 않고** 그 목표로 amovel 을 다시 보내고 도착 판정도 새 목표로 (모드 3 비행 중 갱신, 직선 amovel 만).
      실측(10/12 #130): 출발 직후 다시 보내면 첫 동작 위에 얹혀 실속도가 명령값을 넘을 수 있다 — 부르는 쪽이 횟수·시각을 제한할 것
    - timeout_s 가 None 이면 현재 자세 → 목표 거리와 속도로 계산(motion_timeout_s)"""
    from DSR_ROBOT2 import DR_MV_RA_OVERRIDE, amovel, check_motion, get_current_posx, posx
    if via is not None:
        from DSR_ROBOT2 import amovec
    if path:
        from DSR_ROBOT2 import amovesx
    logger = node.get_logger()
    resent = False
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
            if path:
                pts = [start_pose, *path, pose6]
                timeout_s = sum(motion_timeout_s(a, b, v_lin, floor_s=2.0) for a, b in zip(pts, pts[1:])) + 5.0
            elif via is not None:
                timeout_s = motion_timeout_s(start_pose, via, v_lin) + motion_timeout_s(via, pose6, v_lin)
            else:
                timeout_s = motion_timeout_s(start_pose, pose6, v_lin)
        if path:
            ret = amovesx([posx(*p) for p in path] + [posx(*pose6)], vel=v_cmd, acc=a_cmd)
        elif via is not None:
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
            if retarget is not None and via is None and not path:
                new6 = retarget()
                # 감속 배율이 바뀌었으면 재전송하지 않는다 — 아래에서 멈추고 다시 명령 (원래 속도로 보낸 직후 정지 방지, #166 리뷰)
                if new6 is not None and (node.speed_scale() < 1.0) == slowed:
                    new6 = [float(v) for v in new6]
                    # ra=OVERRIDE: 진행 중 동작을 새 목표로 바꿈. 기본 DUPLICATE 는 앞 동작에 겹쳐 속도가 명령값을 넘는다 (#130 실측 0.52 > 0.45 m/s 의심, #166 리뷰)
                    ret = amovel(posx(*new6), vel=v_cmd, acc=a_cmd, ra=DR_MV_RA_OVERRIDE)
                    if ret not in (None, 0):
                        raise SkillAbort(f"amovel_rejected_{ret}", stopped=stop_robot(node.dsr, logger, "재전송 거부"))
                    cur_rt = get_current_posx()[0]
                    logger.info(f"이동 중 목표 변경 — 새 목표로 다시 보냄 (이전 목표와 {pose_error(new6, pose6)[0]:.1f} mm, 지금 자세에서 {pose_error(cur_rt, new6)[0]:.1f} mm)")
                    pose6 = new6
                    timeout_s = max(timeout_s, (now - t0) + motion_timeout_s(cur_rt, pose6, v_lin))
            if (node.speed_scale() < 1.0) != slowed:
                if not stop_robot(node.dsr, logger, "감속 배율 변경"):
                    raise SkillAbort("stop_failed_on_slow", stopped=False)
                if via is not None or path:
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
                    dpos, dang = pose_error(cur, pose6)
                    # 10/12 실로봇: 공 회수 순응 하강이 바닥 근처(목표 5 mm 앞)까지 왔을 때 손 때문에 SLOW↔NORMAL 이 바뀌어 같은 목표를
                    # 다시 보냈는데 움직이지 않음 (공에 닿지 않았음 — 순응 상태에선 명령점과 실제 TCP 사이에 몇 mm 차이가 남아
                    # 컨트롤러는 이미 도착한 것으로 봄) → not_started 로 회수 포기. 접촉 하강(stall_ok)은 멈춘 자리에서 이어 간다
                    if stopped and stall_ok:
                        logger.info(f"출발 없이 목표 {dpos:.1f} mm 앞 — 순응 하강 끝자리로 보고 여기서 이어 감")
                        return "stalled"
                    if stopped and dpos <= NEAR_DONE_MM and dang <= ARRIVE_TOL_DEG:
                        logger.warning(f"출발 없이 목표 {dpos:.1f} mm 안 — 도착으로 봄")
                        return "done"
                    if not resent and stopped:                            # 10/8 실로봇: 순응 해제 직후 명령 유실 → 한 번은 다시 보낸다
                        resent = True
                        logger.warning(f"{START_GRACE_S:.0f} s 안에 움직임 없음 — 명령 유실로 보고 한 번 다시 보냄 (목표와 {pose_error(cur, pose6)[0]:.1f} mm)")
                        time.sleep(COMPLIANCE_SETTLE_S)
                        break
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


def move_chain(node, vias, final_pose, final_vel, final_acc, on_poll=None, stall_ok=False, final_compliance=None,
               on_via=None, tag="chain", lead_mm=10.0, final_lead_mm=30.0):
    """멈추지 않고 이어 가는 직선 이동 체인 (두산 블렌딩) → 마지막 구간만 느리게(접촉 감지).
    vias = [(pose6, vel, acc, radius_mm), ...] 경유점. 각 경유점에 amovel(radius) 을 보내고, 그 반경에 들어오면 다음 명령을 보낸다
    (10/7 에뮬레이터: 반경 20 mm 면 150 → 40 mm/s 로 정지 없이 이어짐. 스플라인 amovesx 뒤에는 블렌딩이 안 돼 직선 체인을 쓴다).
    마지막 구간(final_pose, final_vel, final_acc)은 move_checked 로 — 접촉(on_poll)·STOP·도착·stall_ok 검사.
    final_compliance 가 있으면 마지막 구간 직전에 **이동 중에** 순응을 켠다(기다리지 않음). 정상 반환 시 순응은 켜진 채이고
    해제는 호출한 쪽 책임 (그리퍼를 순응 상태에서 여닫기 위해). 예외면 여기서 해제한다.
    실로봇에서 순응 직후 명령이 무시된 적이 있다 → 마지막 구간이 출발 못 하고 마지막 경유점에서 서 버리면 접촉으로 보지 않고 한 번 다시 보낸다.
    on_via(i): 경유점 i 반경에 들어왔을 때 호출 (예: 공에서 충분히 떨어진 곳에서 그리퍼 닫기 시작).
    lead_mm / final_lead_mm: 반경보다 이만큼 먼저 다음 명령을 보낸다. 마지막 구간 전환은 순응 켜기·검사로 시간이 더 걸려
      (에뮬레이터 10 mm 일 때 0.2 s 머뭇거림) 더 일찍 보낸다."""
    from DSR_ROBOT2 import amovel, get_current_posx, posx
    logger = node.get_logger()
    comp_on = False
    try:
        for i, (pose, vel, acc, radius) in enumerate(vias):
            if final_compliance is not None and i == len(vias) - 1:
                radius = 0.0                                  # 순응을 켤 마지막 경유점에서는 블렌딩하지 않고 감속해 선다
            if node.stop_requested():
                raise SkillAbort(node.safety_reason(), stopped=stop_robot(node.dsr, logger, "safety STOP"))
            scale = node.speed_scale()
            sc = (lambda v: [float(x) * scale for x in v]) if isinstance(vel, (list, tuple)) else (lambda v: float(v) * scale)
            ret = amovel(posx(*pose), vel=sc(vel), acc=sc(acc), radius=float(radius))
            if ret not in (None, 0):
                raise SkillAbort(f"amovel_rejected_{ret}", stopped=stop_robot(node.dsr, logger, "체인 명령 거부"))
            v_lin = sc(vel)[0] if isinstance(vel, (list, tuple)) else sc(vel)
            deadline = time.monotonic() + motion_timeout_s(get_current_posx()[0], pose, v_lin)
            lead = final_lead_mm if i == len(vias) - 1 else lead_mm
            while pose_error(get_current_posx()[0], pose)[0] > float(radius) + lead:   # 이 경유점 반경(+미리) 안으로
                time.sleep(0.01)
                if node.stop_requested():
                    raise SkillAbort(node.safety_reason(), stopped=stop_robot(node.dsr, logger, "safety STOP"))
                if time.monotonic() > deadline:
                    raise SkillAbort("chain_timeout", stopped=stop_robot(node.dsr, logger, "체인 이동 시간 초과"))
            if on_via:
                on_via(i)
        if final_compliance is not None:
            # 10/8 실로봇: 이동 중에 순응을 켜면 그 직후 명령을 컨트롤러가 자주 버려 섰다 가는 덜컹이 났다 →
            # 마지막 경유점에 멈춘 뒤 켜고 COMPLIANCE_SETTLE_S 기다렸다 마지막 구간을 보낸다 (≈0.5 s 의 의도된 멈춤)
            from DSR_ROBOT2 import check_motion, task_compliance_ctrl
            t_wait = time.monotonic()
            while check_motion() != 0 and time.monotonic() - t_wait < 5.0:
                time.sleep(0.02)
                if node.stop_requested():
                    raise SkillAbort(node.safety_reason(), stopped=stop_robot(node.dsr, logger, "safety STOP"))
            task_compliance_ctrl(list(final_compliance))
            comp_on = True
            time.sleep(COMPLIANCE_SETTLE_S)
        how = move_checked(node, final_pose, final_vel, final_acc, on_poll=on_poll, stall_ok=stall_ok)
        # 순응을 켠 직후의 마지막 구간 명령을 컨트롤러가 버리면 로봇은 앞 구간 끝(경유점 블렌딩 반경 안)에서 선다 → 멈춤이 '접촉'으로 읽힌다.
        # 마지막 경유점을 지나 마지막 구간을 거의 못 내려갔으면(경유점보다 목표에 5 mm 이상 가깝지 않으면) 접촉이 아니라 명령 유실 → 다시 보낸다.
        # (10/8 실로봇: 경유점 위 약 18 mm, 목표 82 mm 앞에서 멈춰 공을 못 집음 — 예전 기준 '경유점 8 mm 안' 에 안 걸렸다)
        if vias:
            seg = pose_error(vias[-1][0], final_pose)[0]
            for attempt in (1, 2):
                if how != "stalled" or pose_error(get_current_posx()[0], final_pose)[0] < seg - 5.0:
                    break
                logger.warning(f"[{tag}] 마지막 구간을 못 내려가고 멈춤 (목표까지 {pose_error(get_current_posx()[0], final_pose)[0]:.0f} mm, "
                               f"구간 {seg:.0f} mm) — 접촉 아님, 다시 보냄 {attempt}/2")
                time.sleep(0.2)
                how = move_checked(node, final_pose, final_vel, final_acc, on_poll=on_poll, stall_ok=stall_ok)
        return how
    except BaseException:
        if comp_on:
            try:
                from DSR_ROBOT2 import release_compliance_ctrl
                release_compliance_ctrl()
            except Exception as e:  # noqa: BLE001
                logger.error(f"release_compliance_ctrl 실패: {e}")
        raise


def joint_path_max_x(q0, q1, fk, samples=20):
    """관절 직선 보간(movej 경로) 위 TCP x 최댓값 mm. fk(posj deg) → posx. 순수 함수 (테스트용)."""
    xs = []
    for i in range(samples + 1):
        t = i / samples
        p = fk([a + (b - a) * t for a, b in zip(q0, q1)])
        if p is None or isinstance(p, (int, float)):
            return None
        xs.append(float(p[0]))
    return max(xs)


def movej_safe(node, posj6, jv, ja, tv, ta, x_max_mm, via_x_mm=300.0, via_z_mm=450.0):
    """관절 이동 전에 경로(관절 직선 보간)를 두산 fkin 으로 미리 재서 TCP 가 x_max_mm 를 넘으면
    먼저 지금 자세 그대로 곧게 뒤·위(x ≤ via_x, z ≥ via_z)로 빠진 뒤 관절 이동.
    10/12 실로봇: 모드 5 내려놓기 자리(손목 J4 −154°·J6 −257°)에서 관측 자세로 바로 가면 손목이 풀리며 TCP 가 x 0.58~0.78 m
    (보라색 선 0.57 너머)로 휘어 workspace_limit STOP. 경유하면 계산상 최대 x 0.53~0.55 m. fkin 이 없으면(fake) 바로 이동."""
    import DSR_ROBOT2
    from DSR_ROBOT2 import get_current_posj, get_current_posx
    fk = getattr(DSR_ROBOT2, "fkin", None)
    worst = None
    if fk is not None:
        try:
            worst = joint_path_max_x([float(v) for v in get_current_posj()], [float(v) for v in posj6], fk)
        except Exception as e:  # noqa: BLE001
            node.get_logger().warning(f"[movej_safe] fkin 실패 ({e}) — 경로 확인 없이 이동")
    if worst is not None and worst > x_max_mm:
        cur = [float(v) for v in get_current_posx()[0]]
        via = [min(cur[0], via_x_mm), cur[1], max(cur[2], via_z_mm), *cur[3:]]
        node.get_logger().info(f"[movej_safe] 관절 경로가 x {worst:.0f} mm 까지 나감 (한계 {x_max_mm:.0f}) → 먼저 곧게 "
                               f"({via[0]:.0f}, {via[1]:.0f}, {via[2]:.0f}) 로 빠짐")
        move_checked(node, via, tv, ta)
    return movej_checked(node, posj6, jv, ja)


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


J3_ABS_MAX_DEG = 148.0       # 컨트롤러 J3 범위 ±150° (2026-10-08 에뮬레이터 알람 "JNT(3) POS LIMIT ... MAX(150.000)"), 여유 2°


def ik_nearest(pose6, cur_posj, j5_max=None):
    """posx → 8개 IK 해 중 지금 관절과 가장 가까운 해 (관절 차이 합 최소). J1·J4·J6 은 ±360° 중 가까운 쪽으로 감는다.
    J3·J5 가 컨트롤러 범위(J3_ABS_MAX_DEG, calib_sweep.J5_ABS_MAX_DEG) 밖인 해는 버린다. 해가 없으면 None."""
    from DSR_ROBOT2 import ikin
    if j5_max is None:
        from rally_robot.calib_sweep import J5_ABS_MAX_DEG as j5_max
    cur = [float(v) for v in cur_posj]
    best, best_cost = None, float("inf")
    for sol in range(8):
        try:
            r = ikin([float(v) for v in pose6], sol)
            q = [float(v) for v in (r[0] if isinstance(r, tuple) else r)]
        except Exception:                                   # noqa: BLE001  도달 불가 해
            continue
        if len(q) != 6 or abs(q[2]) > J3_ABS_MAX_DEG or abs(q[4]) > j5_max:
            continue
        for j in (0, 3, 5):                                 # ±360 범위 관절: 지금 값에 가까운 쪽으로
            while q[j] - cur[j] > 180.0 and q[j] - 360.0 >= -360.0:
                q[j] -= 360.0
            while cur[j] - q[j] > 180.0 and q[j] + 360.0 <= 360.0:
                q[j] += 360.0
        cost = sum(abs(a - b) for a, b in zip(q, cur))
        if cost < best_cost:
            best, best_cost = q, cost
    return best


def movej_to_pose(node, pose6, vel_deg_s, acc_deg_s2, timeout_s=30.0):
    """posx 목표로 관절 이동 (ik_nearest → movej_checked). 툴 자세가 크게 바뀌는 이동(툴 아래 ↔ 수평)을 movel 로 하면
    도중에 손목 특이점(J5≈0)을 지나 컨트롤러가 멈춘다 (2026-10-08 에뮬레이터, 모드 5 제시). 관절 공간에선 J5=0 을 지나도 된다.
    fake(ikin 없음)면 move_checked 로 대신한다."""
    import DSR_ROBOT2
    if not hasattr(DSR_ROBOT2, "ikin"):
        return move_checked(node, pose6, VELOCITY, ACC)
    q = ik_nearest(pose6, DSR_ROBOT2.get_current_posj())
    if q is None:
        raise SkillAbort("ik_failed", stopped=True)
    return movej_checked(node, q, vel_deg_s, acc_deg_s2, timeout_s=timeout_s)


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


# 모든 스킬 공통 속도 — skills.yaml motion: 이 있으면 그 값. 실제 명령 = 이 값 × 안전 속도 배율(move_checked 안에서)
MOTION_DEFAULTS = dict(move_vel_mm_s=150.0, move_acc_mm_s2=150.0, move_ang_vel_deg_s=90.0, move_ang_acc_deg_s2=180.0,
                       contact_vel_mm_s=36.0, contact_acc_mm_s2=72.0, joint_vel_deg_s=60.0, joint_acc_deg_s2=120.0)


def motion_speeds(node, profile=None):
    """skills.yaml motion: → (공중 이동 vel, acc, 접촉 하강 vel, acc, 관절 vel, acc). 공중 이동은 [직선 mm/s, 회전 deg/s] 짝.
    profile="mode5" 면 skills.yaml mode5.speed_scale(0~1) 을 직선·회전·관절 속도·가속에 한꺼번에 곱한다 (10/11 정정인: 모드 5 는
    movel·movej 전부 한 값으로 조절). 접촉 하강(contact_*)은 접촉 감지 기준이라 곱하지 않는다."""
    cfg = getattr(node, "cfg", None) or {}
    m = {**MOTION_DEFAULTS, **(cfg.get("motion") or {})}
    if profile:
        s = min(1.0, max(0.05, float((cfg.get(profile) or {}).get("speed_scale", 1.0))))
        for k in m:
            if not k.startswith("contact_"):
                m[k] = float(m[k]) * s
    return ([m["move_vel_mm_s"], m["move_ang_vel_deg_s"]], [m["move_acc_mm_s2"], m["move_ang_acc_deg_s2"]],
            m["contact_vel_mm_s"], m["contact_acc_mm_s2"], m["joint_vel_deg_s"], m["joint_acc_deg_s2"])


COMPLIANCE_SETTLE_S = 0.3   # 10/7 실로봇: task_compliance_ctrl 직후 곧바로 amovel 을 보내면 알람 없이 무시된다 (0.1 s 만 기다려도 받음)


@contextmanager
def compliance(stx, logger=None, settle_s=None):
    """task_compliance_ctrl(stx) 를 켜고, 블록을 어떻게 빠져나가든(예외·취소 포함) 반드시 해제한다.
    켠 뒤 settle_s(기본 COMPLIANCE_SETTLE_S) 기다린다 — 정지 상태에서 바로 이동 명령을 보내면 컨트롤러가 버린다. 힘 영점도 이 뒤에 읽을 것.
    이동 중(블렌딩 체인 마지막 구간 직전)에 켤 때는 settle_s=0 — 기다리면 앞 이동이 끝나 멈춰 버린다."""
    from DSR_ROBOT2 import task_compliance_ctrl, release_compliance_ctrl
    task_compliance_ctrl(list(stx))
    time.sleep(COMPLIANCE_SETTLE_S if settle_s is None else settle_s)
    try:
        yield
    finally:
        try:
            release_compliance_ctrl()
            time.sleep(COMPLIANCE_SETTLE_S)                  # 10/8 실로봇: 해제 직후 amovel(PickCompliant 들기)이 -1 로 거부됨
        except Exception as e:  # 해제 실패는 로그만 — 호출자의 예외를 덮지 않는다
            if logger:
                logger.error(f"release_compliance_ctrl 실패: {e}")
