"""rally_robot/dsr_helpers.py — 두산 DSR 초기화와 좌표 변환 공통 유틸 (선생님 robot_control.py에서 분리)
담당: 정정인

사용법 (반드시 rclpy.init() 이후, DSR_ROBOT2 import 전에):
    from rally_robot.dsr_helpers import init_dsr, transform_to_base
    dsr_node = init_dsr()                     # DR_init 설정 + dsr 노드 생성
    from DSR_ROBOT2 import movel, movej, ... # 이제 import 가능
"""
import os
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from ament_index_python.packages import get_package_share_directory

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
    """진행 중인 모션 정지. DSR_ROBOT2 에 래퍼가 없어 /dsr01/motion/move_stop 서비스를 직접 부른다.
    stop_mode: 0 QSTOP_STO · 1 QSTOP · 2 SSTOP(소프트, 기본) · 3 HOLD. 성공 여부 반환."""
    from dsr_msgs2.srv import MoveStop
    client = node.create_client(MoveStop, f"/{ROBOT_ID}/motion/move_stop")
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
