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


def test_check_joint_limits():
    """J5 한계(펜던트 관절 범위) 넘는 자세와 IK 실패를 순회 전에 걸러낸다 (2026-10-07 1206 NOT REACHABLE)."""
    pytest.importorskip("scipy")
    from rally_robot.calib_sweep import check_joint_limits
    ok = [0.0, 9.0, 30.0, -7.0, 131.3, -5.0]
    over = [8.0, 21.0, 7.0, -24.0, 141.1, -10.0]
    neg = [0.0, 0.0, 0.0, 0.0, -134.0, 0.0]
    bad = check_joint_limits([(0, ok), (1, over), (2, None), (3, neg)], j5_max=133.0)
    assert [i for i, _ in bad] == [1, 2, 3]
    assert check_joint_limits([(0, over)], j5_max=145.0) == []


def test_push_ball_plan():
    """공 뒤에서 내려가 +x 로 밀고, 끝점은 공 뒤면 + stroke, 높이는 테이블 + 반지름."""
    pytest.importorskip("rally_interfaces")
    from types import SimpleNamespace
    from rally_robot.skills.push_ball import DEFAULTS, _plan
    lanes = [{"name": "center", "start": [0.40, 0.0, 0.0], "dir_deg": 0}]
    goal = SimpleNamespace(lane=0, push_dir_deg=0.0)
    pts, info = _plan(goal, dict(DEFAULTS, push_mode="push", ball_radius_m=0.03), lanes, {"z_m": 0.0144})
    assert pts["behind"][0] == pytest.approx(400 - 80)          # 공 중심 − (반지름 30 + back_off 50)
    assert pts["end"][0] == pytest.approx(400 - 30 + 60)        # 공 뒤면 + stroke 60
    assert pts["behind"][2] == pytest.approx(14.4 + 30)
    assert pts["above"][2] == pytest.approx(14.4 + 100)
    assert pts["end"][1] == pytest.approx(0.0)
    goal90 = SimpleNamespace(lane=0, push_dir_deg=90.0)          # +y 로 밀기
    pts90, _ = _plan(goal90, dict(DEFAULTS, push_mode="push", ball_radius_m=0.03), lanes, {"z_m": 0.0})
    assert pts90["end"][1] > pts90["behind"][1] and pts90["end"][0] == pytest.approx(400)


def test_push_ball_strike_arc():
    """치기 원호: 타격점은 툴 수직·손가락 끝이 테이블 바로 위·앞면이 공 뒤 표면, 백스윙은 뒤쪽·위, 팔로스루는 앞쪽. 모든 점이 같은 원(반지름 L) 위."""
    pytest.importorskip("rally_interfaces")
    from rally_robot.skills.push_ball import DEFAULTS, _strike_points
    cfg = dict(DEFAULTS, ball_radius_m=0.0315)
    sp, tips = _strike_points(cfg, (0.4324, -0.0058), (1.0, 0.0), 0.0144, 0.0)
    r, L = 0.0315, cfg["strike_radius_m"]
    assert tips["mid"][2] == pytest.approx(0.0144 + cfg["strike_tip_height_m"])
    assert tips["mid"][0] == pytest.approx(0.4324 - r - cfg["strike_face_offset_m"])
    assert tips["start"][0] < tips["mid"][0] < tips["end"][0]                 # 뒤 → 타격 → 앞
    assert tips["start"][2] > tips["mid"][2] and tips["end"][2] > tips["mid"][2]   # 타격점이 원호의 최저점
    z_mid = Rotation.from_euler("ZYZ", sp["mid"][3:6], degrees=True).apply([0, 0, 1])
    assert z_mid[2] < -0.99                                                    # 타격 순간 툴 거의 수직 (실측 tool_down 이 3° 틀어짐)
    pivot = np.array(tips["mid"]) - L * z_mid
    for k in ("start", "end"):
        assert float(np.linalg.norm(np.array(tips[k]) - pivot)) == pytest.approx(L, abs=1e-6)
def test_handover_present_pose_and_tilt():
    """제시 자세: 중앙선 − 여유, 테이블 + 높이, 쥔 자세 유지. tilt 는 범위로 클램프되고 베이스 x 축 회전 → 툴 z 축이 y 방향으로 기운다."""
    pytest.importorskip("rally_interfaces")
    from rally_robot.skills.handover import DEFAULTS, present_pose
    cfg = dict(DEFAULTS)
    table = {"centerline_x_m": 0.52, "z_m": 0.0144}
    cur = [300.0, 0.0, 400.0, 0.0, 180.0, 0.0]                     # 툴 수직 아래
    pose, tilt = present_pose(cfg, table, cur, 0.0)
    assert pose[0] == pytest.approx(460.0) and pose[1] == pytest.approx(0.0) and pose[2] == pytest.approx(234.4)
    pose30, tilt30 = present_pose(cfg, table, cur, 30.0)
    assert tilt30 == 30.0 and pose30[:3] == pose[:3]
    z_axis = Rotation.from_euler("ZYZ", pose30[3:6], degrees=True).apply([0, 0, 1])
    assert math.degrees(math.acos(-z_axis[2])) == pytest.approx(30.0, abs=0.1)   # 수직에서 30° 기울어짐
    assert abs(z_axis[0]) < 1e-6                                                   # x 축 회전이라 x 성분 없음
    _, tilt_c = present_pose(cfg, table, cur, 80.0)
    assert tilt_c == 45.0                                                          # 범위 클램프
