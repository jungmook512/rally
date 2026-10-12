import numpy as np
import pytest

from rally_robot.receive_watch import jaw_roi, near_fraction

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
