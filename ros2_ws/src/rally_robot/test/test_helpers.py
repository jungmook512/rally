"""로봇 없이 도는 단위 테스트: 좌표 변환·툴 회전·후퇴 방향·가짜 DSR 흐름.
CI(ros:jazzy-ros-base)에는 scipy 가 없다 → 그 환경에서는 건너뛴다. 로봇·인식 PC 에서는 전부 돈다."""
import math

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
from scipy.spatial.transform import Rotation  # noqa: E402


def test_posx_pose_round_trip():
    from rally_robot.dsr_helpers import posx_to_pose, pose_to_posx
    posx = [400.0, -120.5, 250.0, 30.0, 120.0, -45.0]
    pose = posx_to_pose(posx)
    assert pose.position.x == pytest.approx(0.4) and pose.position.z == pytest.approx(0.25)
    q = pose.orientation
    back = pose_to_posx((pose.position.x, pose.position.y, pose.position.z), (q.x, q.y, q.z, q.w))
    assert np.allclose(back[:3], posx[:3], atol=1e-9)
    assert np.allclose(Rotation.from_euler("ZYZ", back[3:], degrees=True).as_matrix(),
                       Rotation.from_euler("ZYZ", posx[3:], degrees=True).as_matrix(), atol=1e-9)


def test_rotate_tool_is_tool_frame_rotation():
    from rally_robot.dsr_helpers import rotate_tool
    base = [450.0, 0.0, 350.0, 0.0, 180.0, 0.0]
    out = rotate_tool(base, (1, 0, 0), 12.0)
    assert out[:3] == base[:3]
    d = Rotation.from_euler("ZYZ", base[3:], degrees=True).inv() * Rotation.from_euler("ZYZ", out[3:], degrees=True)
    assert np.allclose(d.as_rotvec(degrees=True), [12.0, 0.0, 0.0], atol=1e-6)


def test_retreat_moves_toward_base():
    from rally_robot.skills.handover import _retreat_pose
    p = _retreat_pose([500.0, 300.0, 300.0, 0.0, 180.0, 0.0], 0.10)
    assert math.hypot(p[0], p[1]) == pytest.approx(math.hypot(500.0, 300.0) - 100.0)
    assert p[2:] == [300.0, 0.0, 180.0, 0.0]


def test_fake_dsr_motion_and_contact():
    from rally_robot import fake_dsr as f
    f.S.pose = [400.0, 0.0, 350.0, 0.0, 180.0, 0.0]
    f.S.target = list(f.S.pose)
    f.amovel([400.0, 0.0, 240.0, 0.0, 180.0, 0.0], vel=60)
    assert f.check_motion() == 1
    f.mwait()
    assert f.check_motion() == 0 and f.get_current_posx()[0][2] == 240.0
    assert f.get_tool_force()[2] < -5.0          # TABLE_Z(250) 아래에서 멈춤 → 접촉 힘
