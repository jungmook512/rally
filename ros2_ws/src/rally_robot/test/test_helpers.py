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
    """제시 자세: 중앙선 − 여유, 테이블 + 높이, 툴 z = 베이스 +x(수평), 툴 +y(카메라) 위. tilt 는 ±90° 클램프, 베이스 x 축 회전."""
    pytest.importorskip("rally_interfaces")
    from rally_robot.skills.handover import DEFAULTS, present_pose
    cfg = dict(DEFAULTS)
    table = {"centerline_x_m": 0.52, "z_m": 0.0144}
    cur = [300.0, 0.0, 400.0, 0.0, 180.0, 0.0]                     # 툴 수직 아래 (집은 자세)
    pose, tilt = present_pose(cfg, table, cur, 0.0)
    assert pose[0] == pytest.approx(460.0) and pose[1] == pytest.approx(0.0) and pose[2] == pytest.approx(304.4)
    m = Rotation.from_euler("ZYZ", pose[3:6], degrees=True).as_matrix()
    assert m[:, 2] == pytest.approx([1, 0, 0], abs=1e-6)                         # 툴 z = 베이스 +x (테이블과 수평, 환자 쪽)
    assert m[:, 1] == pytest.approx([0, 0, 1], abs=1e-6)                         # 툴 +y(카메라 쪽) = 위
    pose30, tilt30 = present_pose(cfg, table, cur, 30.0)
    assert tilt30 == 30.0 and pose30[:3] == pose[:3]
    m30 = Rotation.from_euler("ZYZ", pose30[3:6], degrees=True).as_matrix()
    assert m30[:, 2] == pytest.approx([1, 0, 0], abs=1e-6)                       # 기울여도 툴 z 는 그대로 (x 축 회전)
    assert math.degrees(math.acos(m30[2, 1])) == pytest.approx(30.0, abs=0.1)    # 카메라 쪽이 위에서 30° 기울어짐
    _, tilt_c = present_pose(cfg, table, cur, 120.0)
    assert tilt_c == 90.0                                                         # 범위 클램프



def test_push_ball_speed_for_target():
    """레시피 d = v²/(2a) + d0 의 역: 목표까지 레인 방향 거리 d → v = sqrt(2a(d - d0)). 레시피 없음·너무 가까움은 거부."""
    pytest.importorskip("rally_interfaces")
    from types import SimpleNamespace
    from rally_robot.dsr_helpers import SkillAbort
    from rally_robot.skills.push_ball import speed_for_target
    lane = {"start": [0.43, 0.0, 0.0]}
    goal = SimpleNamespace(target=SimpleNamespace(point=SimpleNamespace(x=0.43 + 0.40, y=0.0)))
    v, d = speed_for_target(goal, lane, {"decel_mps2": 0.5, "d0_m": 0.10}, 0.0)
    assert d == pytest.approx(0.40) and v == pytest.approx(math.sqrt(2 * 0.5 * 0.30))
    with pytest.raises(SkillAbort):
        speed_for_target(goal, lane, None, 0.0)
    near = SimpleNamespace(target=SimpleNamespace(point=SimpleNamespace(x=0.43 + 0.05, y=0.0)))
    with pytest.raises(SkillAbort):
        speed_for_target(near, lane, {"decel_mps2": 0.5, "d0_m": 0.10}, 0.0)


def test_push_ball_choose_ball_smallest_x():
    """비전 공 고르기: ball·신뢰도·정지·x 범위를 통과한 것 중 x 최소. base_link 가 아니면 None."""
    pytest.importorskip("rally_interfaces")
    from types import SimpleNamespace as NS
    from rally_robot.skills.push_ball import DEFAULTS, choose_ball
    def obj(cls, x, y=0.0, conf=0.9, moving=False):
        return NS(class_name=cls, confidence=conf, is_moving=moving, position=NS(x=x, y=y, z=0.05))
    arr = NS(header=NS(frame_id="base_link"), objects=[
        obj("ball", 0.45, 0.1), obj("ball", 0.31, 0.38), obj("ball", 0.28, 0.38, conf=0.3),   # 신뢰도 낮음 → 제외
        obj("ball", 0.25, 0.0, moving=True), obj("cup", 0.20), obj("ball", 0.05),             # 움직임·컵·범위 밖 제외
        NS(class_name="ball", confidence=0.99, is_moving=False, position=NS(x=0.20, y=0.0, z=0.51))])   # 그리퍼에 쥔 공(높이) 제외
    x, y, z, n = choose_ball(arr, dict(DEFAULTS))
    assert (x, y, n) == (0.31, 0.38, 2)
    assert choose_ball(NS(header=NS(frame_id="camera_color_optical_frame"), objects=arr.objects), dict(DEFAULTS)) is None


def test_push_ball_hand_view_pose():
    """친 뒤 손 보기 자세: 툴 z 가 진행 방향(+x)으로 tilt 만큼 기울고, yaw 90(집을 때 방향)이면 카메라 쪽(툴 +y)이 위."""
    pytest.importorskip("rally_interfaces")
    from rally_robot.skills.push_ball import DEFAULTS, _hand_view_pose
    cfg = dict(DEFAULTS, tool_down_zyz=[174.8, -177.0, 175.1], hand_view_tilt_deg=45.0)
    pose = _hand_view_pose(cfg, (0.43, 0.0), (1.0, 0.0), 0.0144, 90.0)
    m = Rotation.from_euler("ZYZ", pose[3:6], degrees=True).as_matrix()
    assert m[0, 2] > 0.6                                                   # 툴 z 가 환자 쪽(+x)
    assert math.degrees(math.acos(-m[2, 2])) == pytest.approx(45.0, abs=4.0)   # 수직에서 ≈45° (tool_down 이 3° 기울어 있음)
    assert m[2, 1] > 0.6                                                   # 카메라 쪽(툴 +y)이 위
    assert pose[0] == pytest.approx(330.0) and pose[2] == pytest.approx(314.4)


def test_safety_quat_angle():
    """nudge '움직이는 중' 판정용 툴 회전각: 같은 자세 0°, z 축 90° 회전 90°, 부호 반대 쿼터니언(같은 자세)도 0°."""
    pytest.importorskip("rally_interfaces")
    from rally_robot.safety_monitor import quat_angle_deg
    s = math.sqrt(0.5)
    assert quat_angle_deg((0, 0, 0, 1), (0, 0, 0, 1)) == pytest.approx(0.0, abs=1e-6)
    assert quat_angle_deg((0, 0, 0, 1), (0, 0, s, s)) == pytest.approx(90.0, abs=1e-4)
    assert quat_angle_deg((0, 0, s, s), (0, 0, -s, -s)) == pytest.approx(0.0, abs=1e-4)


def test_return_ball_lane_candidates():
    """공 회수: 레인 통로 안·정지 공만, 로봇이 닿는 곳(x ≤ 0.55)과 아닌 곳을 나눔. 거치대 공(레인 밖)은 무시."""
    pytest.importorskip("rally_interfaces")
    from rally_interfaces.msg import TrackedObject, TrackedObjectArray
    from rally_robot.skills.push_ball import DEFAULTS as PD
    from rally_robot.skills.return_ball import DEFAULTS, lane_ball_candidates
    cfg = {**PD, **DEFAULTS, "vision_offset_m": [0.0, 0.0, 0.0]}
    lanes = [{"start": [0.43, 0.14, 0.0], "dir_deg": 0.0}, {"start": [0.43, 0.0, 0.0], "dir_deg": 0.0},
             {"start": [0.43, -0.15, 0.0], "dir_deg": 0.0}]
    arr = TrackedObjectArray()
    arr.header.frame_id = "base_link"

    def ball(x, y, moving=False):
        o = TrackedObject(class_name="ball", confidence=0.9, is_moving=moving)
        o.position.x, o.position.y, o.position.z = x, y, 0.046
        return o
    arr.objects = [ball(0.50, 0.01), ball(0.70, -0.14), ball(0.334, 0.282), ball(0.52, 0.15, moving=True)]
    lane_cfg = {**cfg, "return_zone_enabled": False}
    near, far = lane_ball_candidates(arr, lane_cfg, lanes, -1, 0.0144)
    assert near == [(0.50, 0.01, 0.046)] and far == [(0.70, -0.14, 0.046)]       # 거치대 공·움직이는 공 제외
    near, far = lane_ball_candidates(arr, lane_cfg, lanes, 2, 0.0144)
    assert near == [] and far == [(0.70, -0.14, 0.046)]                           # 레인 2 만
    # 회수 구역 (기본, 10/8): 레인 밖·레인 입구 뒤로 굴러온 공도 집음. 거치대·너무 가까운 공은 무시
    arr.objects = [ball(0.38, 0.08), ball(0.30, -0.25), ball(0.25, 0.0), ball(0.334, 0.282), ball(0.70, -0.14)]
    near, far = lane_ball_candidates(arr, cfg, lanes, 2, 0.0144)
    assert near == [(0.30, -0.25, 0.046), (0.38, 0.08, 0.046)] and far == [(0.70, -0.14, 0.046)]


def test_hand_view_pose_is_lane_center():
    """손 보기 자세는 레인과 상관없이 세 레인 가운데 기준 하나."""
    pytest.importorskip("rally_interfaces")
    from rally_robot.skills.push_ball import DEFAULTS, hand_view_pose
    cfg = dict(DEFAULTS, tool_down_zyz=[174.8, -177.0, 175.1], push_yaw_deg=90.0)
    lanes = [{"start": [0.43, 0.14, 0.0]}, {"start": [0.43, 0.0, 0.0]}, {"start": [0.43, -0.14, 0.0]}]
    pose = hand_view_pose(cfg, lanes, {"z_m": 0.0144})
    assert pose[0] == pytest.approx(330.0) and pose[1] == pytest.approx(0.0, abs=1e-6)
    assert hand_view_pose(cfg, [], {}) is None


def test_push_ball_stop_measure_helpers():
    """정지 위치: 레인 통로 안·입구에서 5 cm 이상 간 공만, 가장 멀리 간 공 먼저. StopWatch 는 0.8 s 동안 1 cm 안이면 멈춤."""
    pytest.importorskip("rally_interfaces")
    from rally_interfaces.msg import TrackedObject, TrackedObjectArray
    from rally_robot.skills.push_ball import DEFAULTS, StopWatch, balls_in_lane
    cfg = dict(DEFAULTS, vision_offset_m=[0.0, 0.0, 0.0])
    lane = {"start": [0.43, 0.0, 0.0]}
    arr = TrackedObjectArray()
    arr.header.frame_id = "base_link"

    def ball(x, y):
        o = TrackedObject(class_name="ball", confidence=0.9)
        o.position.x, o.position.y, o.position.z = x, y, 0.046
        return o
    arr.objects = [ball(0.45, 0.0), ball(0.80, 0.02), ball(0.70, 0.15), ball(0.334, 0.282)]
    got = balls_in_lane(arr, cfg, lane, 0.0, 0.0144)
    assert [round(b[0], 3) for b in got] == [0.37]          # 입구 2 cm(무시)·옆 레인·거치대 제외
    w = StopWatch(0.8, 0.01)
    assert w.add(0.0, (0.70, 0.0)) is None
    assert w.add(0.3, (0.75, 0.0)) is None                  # 아직 구름 → 기준 다시
    assert w.add(0.9, (0.752, 0.0)) is None                 # 0.6 s
    assert w.add(1.2, (0.751, 0.0)) == (0.75, 0.0)          # 0.9 s 그대로 → 멈춤
    assert w.add(1.3, None) is None                         # 안 보이면 처음부터


def test_grasp_wrist_nan_when_unknown():
    import math
    from rally_interfaces.msg import HandState
    from rally_robot.skills.handover import grasp_wrist_deg
    h = HandState(detected=False, wrist_angle_deg=35.0)
    h.header.stamp.sec = 100
    now = 100 * 1_000_000_000 + 200_000_000
    assert grasp_wrist_deg(h, now) == 35.0                    # 가까워서 detected=false 여도 필터가 유지한 값
    assert math.isnan(grasp_wrist_deg(None, now))
    assert math.isnan(grasp_wrist_deg(h, now + 2_000_000_000))  # 오래된 메시지


def test_rack_drop_approach_is_up_and_back():
    # 10/11: 놓는 자리 바로 위가 아니라 위 6 cm · −x 6 cm 에서 대각선으로 (자세는 그대로)
    from rally_robot.skills.return_ball import rack_drop_approach
    drop = [462.5, 289.8, 76.3, -171.6, -157.3, -92.0]
    assert rack_drop_approach(drop, 0.06, 0.06) == pytest.approx([402.5, 289.8, 136.3, -171.6, -157.3, -92.0])
    assert rack_drop_approach(drop, 0.06, 0.0) == pytest.approx([462.5, 289.8, 136.3, -171.6, -157.3, -92.0])   # 예전 (수직)


def test_grip_rotation_perpendicular_to_mask_axis():
    # 10/11: 손가락(기준 자세에서 베이스 x 로 닫힘)이 물체 주축에 수직이 되게 베이스 Z 회전, −90~90
    import math
    from rally_robot.skills.pick_compliant import grip_rotation_deg
    assert grip_rotation_deg(math.radians(90)) == pytest.approx(0)      # 막대 y 방향 → 그대로 (10/8 수동과 같음)
    assert grip_rotation_deg(math.radians(-90)) == pytest.approx(0)     # 주축 ±π 모호해도 같음
    assert abs(grip_rotation_deg(math.radians(0))) == pytest.approx(90)  # 막대 x 방향 → 90° 돌림
    assert grip_rotation_deg(math.radians(45)) == pytest.approx(-45)    # 손가락 방향 = 0 − 45 = −45° ⟂ 45°
    assert grip_rotation_deg(math.radians(45 + 180)) == pytest.approx(-45)
    assert grip_rotation_deg(math.radians(30), finger_axis_deg=90) == pytest.approx(30)   # 손가락이 y 로 닫히는 그리퍼면


def test_motion_speeds_profile_scale():
    # 10/11: 모드 5 는 mode5.speed_scale 하나로 직선·회전·관절 속도·가속을 한꺼번에, 접촉 하강은 그대로
    from types import SimpleNamespace as NS
    from rally_robot.dsr_helpers import motion_speeds
    node = NS(cfg={"motion": {"joint_vel_deg_s": 60.0, "move_vel_mm_s": 150.0, "contact_vel_mm_s": 36.0},
                   "mode5": {"speed_scale": 0.5}})
    assert motion_speeds(node)[4] == 60.0
    tv, ta, cv, _, jv, ja = motion_speeds(node, "mode5")
    assert (jv, tv[0], cv) == (30.0, 75.0, 36.0) and ja == 60.0


def test_joint_path_max_x_samples_interpolated_path():
    from rally_robot.dsr_helpers import joint_path_max_x
    # 가짜 fk: x = 500 + 100·sin(J1 보간 비율·π) → 가운데서 600 (끝점만 보면 놓침)
    import math

    def fk(q):
        return [500.0 + 100.0 * math.sin(math.pi * q[0] / 90.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    assert joint_path_max_x([0.0] * 6, [90.0] + [0.0] * 5, fk) == pytest.approx(600.0, abs=1.0)
    assert joint_path_max_x([0.0] * 6, [90.0] + [0.0] * 5, lambda q: -1) is None    # fkin 실패


def test_move_checked_retarget_switches_goal_without_stopping():
    """모드 3 비행 중 갱신: 이동 중 retarget() 이 새 목표를 주면 멈추지 않고 그쪽으로 가서 새 목표에 도착한다 (가짜 DSR)."""
    import sys
    from types import SimpleNamespace
    pytest.importorskip("rclpy")
    from rally_robot import fake_dsr as f
    sys.modules["DSR_ROBOT2"] = f
    from rally_robot.dsr_helpers import move_checked
    f.S.pose = [400.0, 0.0, 350.0, 0.0, 180.0, 0.0]
    f.S.target = list(f.S.pose)
    logs = []
    log = SimpleNamespace(info=logs.append, warning=logs.append, error=logs.append)
    node = SimpleNamespace(stop_requested=lambda: False, speed_scale=lambda: 1.0, safety_reason=lambda: "", dsr=f,
                           get_logger=lambda: log)
    first = [400.0, -100.0, 350.0, 0.0, 180.0, 0.0]
    second = [400.0, -150.0, 400.0, 0.0, 180.0, 0.0]
    calls = {"n": 0}

    def retarget():
        calls["n"] += 1
        return second if calls["n"] == 3 else None            # 몇 바퀴 돈 뒤(이동 중) 한 번만
    assert move_checked(node, first, 450.0, 1000.0, retarget=retarget) == "done"
    assert f.get_current_posx()[0][:3] == pytest.approx(second[:3])
    assert any("이동 중 목표 변경" in m for m in logs)
    assert f.S.last_ra == f.DR_MV_RA_OVERRIDE                 # #166 리뷰: 재전송은 겹치기(DUPLICATE) 아님


def test_move_checked_retarget_skipped_when_speed_changes():
    """#166 리뷰: 같은 바퀴에 감속 배율이 바뀌면 재전송하지 않고 멈춘 뒤 같은 목표로 다시 명령."""
    import sys
    from types import SimpleNamespace
    pytest.importorskip("rclpy")
    from rally_robot import fake_dsr as f
    sys.modules["DSR_ROBOT2"] = f
    from rally_robot.dsr_helpers import move_checked
    f.S.pose = [400.0, 0.0, 350.0, 0.0, 180.0, 0.0]
    f.S.target = list(f.S.pose)
    logs = []
    log = SimpleNamespace(info=logs.append, warning=logs.append, error=logs.append)
    scale = {"v": 1.0}
    node = SimpleNamespace(stop_requested=lambda: False, speed_scale=lambda: scale["v"], safety_reason=lambda: "", dsr=None,  # None = 가짜 정지
                           get_logger=lambda: log)
    first = [400.0, -100.0, 350.0, 0.0, 180.0, 0.0]
    second = [400.0, -150.0, 400.0, 0.0, 180.0, 0.0]
    calls = {"n": 0}

    def retarget():
        calls["n"] += 1
        if calls["n"] == 3:
            scale["v"] = 0.5                                  # 이 바퀴에 SLOW 로 바뀜
            return second
        return None
    move_checked(node, first, 450.0, 1000.0, retarget=retarget)
    assert not any("이동 중 목표 변경" in m for m in logs)
    assert any("안전 레벨 변경" in m for m in logs)
