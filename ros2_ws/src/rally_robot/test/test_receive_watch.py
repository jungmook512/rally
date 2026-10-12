from types import SimpleNamespace as NS

import numpy as np
import pytest

from rally_robot.receive_watch import jaw_roi, near_fraction, object_seen

K = [[909.5, 0.0, 659.5], [0.0, 909.2, 370.2], [0.0, 0.0, 1.0]]
# 10/12 실측 T_gripper2camera 와 같은 꼴: 카메라가 TCP 뒤 234 mm, 툴 y 76 mm (그리퍼 → 카메라 축 x·y 반대)
G2C = np.array([[-1.0, 0.0, 0.0, 33.1], [0.0, -1.0, 0.0, 75.7], [0.0, 0.0, 1.0, -233.6], [0.0, 0.0, 0.0, 1.0]])
BOX = [-35.0, 35.0, -15.0, 60.0, -40.0, 5.0]


def test_jaw_roi_is_bottom_center_and_clipped():
    roi = jaw_roi(K, G2C, BOX, (720, 1280))
    assert roi is not None
    u0, v0, u1, v1 = roi
    assert 600 < u0 < u1 < 1100          # 가운데에서 약간 오른쪽 (카메라 x 오프셋 33 mm)
    assert 400 < v0 < v1 == 720          # 화면 아래쪽 (그리퍼 위 6 cm 까지 포함), 아래 가장자리에서 잘림


def test_jaw_roi_none_when_behind_camera():
    assert jaw_roi(K, G2C, [-10, 10, -10, 10, -400, -300], (720, 1280)) is None


def test_near_fraction_counts_zero_and_close():
    d = np.full((10, 10), 1200, np.uint16)
    d[:5, :] = 0                          # 너무 가까워 못 잰 픽셀
    d[5:7, :] = 250
    assert near_fraction(d, (0, 0, 10, 10), 300.0) == pytest.approx(0.7)
    assert near_fraction(d, (0, 7, 10, 10), 300.0) == 0.0



def test_rod_image_tilt_pairs_objects_with_axes():
    import math

    from geometry_msgs.msg import Pose, PoseArray, Quaternion

    from rally_interfaces.msg import TrackedObject, TrackedObjectArray
    from rally_robot.receive_watch import rod_image_tilt

    def q(deg):
        a = math.radians(deg)
        return Quaternion(z=math.sin(a / 2), w=math.cos(a / 2))
    objs = TrackedObjectArray()
    objs.header.stamp.sec = 5
    objs.objects = [TrackedObject(class_name="cup", confidence=0.9), TrackedObject(class_name="rod", confidence=0.3),
                    TrackedObject(class_name="rod", confidence=0.6), TrackedObject(class_name="rod", confidence=0.9)]
    axes = PoseArray()
    axes.header.stamp.sec = 5
    axes.poses = [Pose(orientation=q(10)), Pose(orientation=q(-40)), Pose(orientation=q(30)), Pose(orientation=Quaternion(w=0.0))]
    assert rod_image_tilt(objs, axes) == pytest.approx(30.0)          # 둥근(축 없음) 0.9 는 빼고 신뢰도 높은 막대
    assert rod_image_tilt(objs, axes, min_conf=0.7) is None
    axes.header.stamp.sec = 6
    assert rod_image_tilt(objs, axes) is None                         # 다른 프레임이면 안 씀


def test_median_recent_and_tilt_step():
    from rally_robot.receive_watch import median_recent, tilt_step
    s = [(0.0, 5.0), (0.5, 20.0), (0.8, 22.0), (0.9, 90.0)]
    assert median_recent(s, 1.0, 1.0, 3) == 22.0
    assert median_recent(s, 1.0, 0.3, 3) is None
    assert tilt_step(0.0, 8.0, 10.0, 20.0) is None
    assert tilt_step(0.0, 50.0, 10.0, 20.0) == 20.0
    assert tilt_step(30.0, 15.0, 10.0, 20.0) == 15.0


def test_object_seen_needs_class_and_confidence():
    objs = NS(objects=[NS(class_name="cup", confidence=0.9), NS(class_name="rod", confidence=0.2)])
    assert object_seen(objs, "cup")
    assert not object_seen(objs, "rod")                 # 신뢰도 0.2 < 0.25
    assert object_seen(objs, "rod", min_conf=0.1)
    assert not object_seen(NS(objects=[]), "rod")       # 아무것도 없음 (손가락만) → 깊이로 안 닫음
    assert not object_seen(None, "rod")                 # 비전 없음


def test_wrap90_and_axis_mean_handle_horizontal_rod():
    from rally_robot.receive_watch import axis_mean, wrap90
    assert wrap90(100.0) == pytest.approx(-80.0) and wrap90(-95.0) == pytest.approx(85.0) and wrap90(90.0) == 90.0
    assert wrap90(-90.0) == 90.0
    flips = [(0.0, -89.0), (0.1, 88.0), (0.2, -87.0), (0.3, 89.0)]      # 가로 막대: −90·+90 이 섞여 들어옴
    m = axis_mean(flips, 0.4, 1.0, 2)
    assert abs(abs(m) - 90.0) < 3.0                                        # 0 이 아니라 ±90 근처
    assert axis_mean([(0.0, 20.0), (0.1, 30.0)], 0.2, 1.0, 2) == pytest.approx(25.0, abs=0.5)
    assert axis_mean([(0.0, 20.0)], 0.2, 1.0, 2) is None


def test_approach_plan_moves_only_along_tool_z_and_respects_limits():
    from rally_robot.receive_watch import approach_plan

    tcp = [479.0, -150.0, 400.0, 0.0, 90.0, 90.0]      # 받기 자리: 툴 z = +x, 손가락 닫힘 = y, 막대(기울기 0) = z
    args = dict(jaw_depth_m=0.02, max_m=0.10, min_m=0.01, finger_max_m=0.03, along_rod_max_m=0.10, x_limit_m=0.538)
    adv, _, side = approach_plan(tcp, (0.510, -0.150, 0.40), **args)
    assert adv == pytest.approx(0.051, abs=1e-6)       # 3.1 cm 앞 + 2 cm 지나가게
    assert side == pytest.approx((0.0, 0.0), abs=1e-6)
    assert approach_plan(tcp, (0.510, -0.130, 0.45), **args)[2] == pytest.approx((0.02, 0.05), abs=1e-6)   # 옆 2 cm · 위 5 cm 도 옮김
    assert approach_plan(tcp, (0.510, -0.150, 0.48), **args)[0] == pytest.approx(0.051, abs=1e-6)   # 막대 방향(위)으로 8 cm 는 괜찮음
    assert approach_plan(tcp, (0.520, -0.150, 0.40), **args)[0] is None      # 다가가면 보라색 선 − 3 cm 넘음
    assert approach_plan(tcp, (0.510, -0.110, 0.40), **args)[0] is None      # 손가락 닫히는 방향으로 4 cm 벗어남
    assert approach_plan(tcp, (0.510, -0.150, 0.52), **args)[0] is None      # 막대 방향으로 12 cm 벗어남
    assert approach_plan(tcp, (0.465, -0.150, 0.40), **args)[0] == 0.0       # 이미 그리퍼 사이
    adv, _, side = approach_plan(tcp, (0.465, -0.150, 0.44), **args)         # 앞은 충분, 위로 4 cm 어긋남 → 위로만
    assert adv == 0.0 and side == pytest.approx((0.0, 0.04), abs=1e-6)
    far = dict(args, x_limit_m=1.0)
    assert approach_plan(tcp, (0.600, -0.150, 0.40), **far)[0] is None       # 12 cm 앞 — 한계 10 cm 보다 멂


def test_recent_position_and_median_xyz():
    from geometry_msgs.msg import Point

    from rally_interfaces.msg import TrackedObject, TrackedObjectArray
    from rally_robot.receive_watch import median_xyz, recent_position

    arr = TrackedObjectArray()
    arr.header.frame_id = "base_link"
    arr.objects = [TrackedObject(class_name="rod", confidence=0.5, position=Point(x=0.5, y=-0.1, z=0.4)),
                   TrackedObject(class_name="rod", confidence=0.9, position=Point(x=0.0, y=0.0, z=0.0)),   # 깊이 못 잼
                   TrackedObject(class_name="cup", confidence=0.95, position=Point(x=0.3, y=0.0, z=0.1))]
    assert recent_position(arr, "rod") == pytest.approx((0.5, -0.1, 0.4))
    arr.header.frame_id = "camera_color_optical_frame"
    assert recent_position(arr, "rod") is None                                 # base 좌표가 아니면 안 씀
    s = [(0.0, (0.5, 0.0, 0.4)), (0.1, (0.6, 0.0, 0.4)), (0.2, (0.52, 0.0, 0.4)), (-5.0, (9.0, 9.0, 9.0))]
    assert median_xyz(s, 0.3, 0.6, 3) == pytest.approx((0.52, 0.0, 0.4))
    assert median_xyz(s, 0.3, 0.6, 4) is None


def test_cup_rim_point_and_class_near():
    from geometry_msgs.msg import Point

    from rally_interfaces.msg import TrackedObject, TrackedObjectArray
    from rally_robot.skills.pick_compliant import class_near, rim_point

    assert rim_point(487.0, -261.0, 42.0, 0.0) == pytest.approx((445.0, -261.0))   # 손가락 x 방향, 로봇 쪽 벽
    assert rim_point(487.0, -261.0, 42.0, 90.0) == pytest.approx((487.0, -219.0))  # 손가락 y 방향이면 원점에 가까운 쪽
    arr = TrackedObjectArray()
    arr.header.frame_id = "base_link"
    arr.objects = [TrackedObject(class_name="rod", position=Point(x=0.38, y=-0.16)),
                   TrackedObject(class_name="cup", position=Point(x=0.49, y=-0.26))]
    assert class_near(arr, 0.487, -0.261, 0.06) == "cup"
    assert class_near(arr, 0.2, 0.2, 0.06) == ""
