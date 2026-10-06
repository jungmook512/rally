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
    from DSR_ROBOT2 import check_motion
    for attempt, mode in enumerate(STOP_MODES, 1):
        ok = move_stop(dsr_node, stop_mode=mode) if dsr_node is not None else True
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


def move_checked(node, pose6, vel, acc, timeout_s=None, poll_s=0.02, on_poll=None):
    """amovel + check_motion 폴링. node 는 stop_requested()/speed_scale()/dsr/get_logger() 를 제공해야 한다.
    timeout_s 가 None 이면 현재 자세 → 목표 거리와 속도로 계산(motion_timeout_s).
    - STOP → 정지 확인 후 SkillAbort(stopped=확인 여부)
    - 감속 배율이 바뀌면 정지 확인 후 같은 자세로 재명령
    - on_poll() 이 True 를 돌려주면(예: 접촉 감지) 정지하고 'interrupted' 를 돌려준다. 정상 도착이면 'done'."""
    from DSR_ROBOT2 import amovel, check_motion, get_current_posx, posx
    logger = node.get_logger()
    while True:
        if node.stop_requested():
            raise SkillAbort(node.safety_reason(), stopped=True)
        scale = node.speed_scale()
        slowed = scale < 1.0
        if timeout_s is None:
            timeout_s = motion_timeout_s(get_current_posx()[0], pose6, vel * scale)
        amovel(posx(*pose6), vel=vel * scale, acc=acc * scale)
        t0 = time.monotonic()
        while check_motion() != 0:
            time.sleep(poll_s)
            if node.stop_requested():
                raise SkillAbort(node.safety_reason(), stopped=stop_robot(node.dsr, logger, "safety STOP"))
            if on_poll is not None and on_poll():
                if not stop_robot(node.dsr, logger, "조건 충족(접촉)"):
                    raise SkillAbort("stop_failed_on_contact", stopped=False)
                return "interrupted"
            if (node.speed_scale() < 1.0) != slowed:
                if not stop_robot(node.dsr, logger, "감속 배율 변경"):
                    raise SkillAbort("stop_failed_on_slow", stopped=False)
                logger.info(f"안전 레벨 변경(speed_override={node.speed_scale():.2f}) — 같은 자세로 재이동")
                break
            if time.monotonic() - t0 > timeout_s:
                stopped = stop_robot(node.dsr, logger, "이동 타임아웃")
                raise SkillAbort(f"motion_timeout_{timeout_s:.0f}s", stopped=stopped)
        else:
            return "done"


@contextmanager
def compliance(stx, logger=None):
    """task_compliance_ctrl(stx) 를 켜고, 블록을 어떻게 빠져나가든(예외·취소 포함) 반드시 해제한다."""
    from DSR_ROBOT2 import task_compliance_ctrl, release_compliance_ctrl
    task_compliance_ctrl(list(stx))
    try:
        yield
    finally:
        try:
            release_compliance_ctrl()
        except Exception as e:  # 해제 실패는 로그만 — 호출자의 예외를 덮지 않는다
            if logger:
                logger.error(f"release_compliance_ctrl 실패: {e}")
