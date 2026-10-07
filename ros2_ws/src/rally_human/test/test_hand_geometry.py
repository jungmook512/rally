"""hand_geometry · gesture 단위 테스트 — 가짜 손 21점으로 쥔 정도·제스처·손바닥 좌표계·손목 회전·중앙선 거리."""
import math

import numpy as np
import pytest

from rally_human import hand_geometry as hg
from rally_human.gesture import GestureFilter

SEG = 0.025          # 손가락 마디 길이 [m]


def make_hand(curl_deg: dict[str, float] | float = 0.0, right: bool = True) -> np.ndarray:
    """손바닥이 아래(−z)를 보고 손가락이 앞(+y)을 향한 손. curl_deg 는 관절마다 아래로 꺾는 각.
    오른손은 엄지·검지가 왼쪽(−x), 왼손은 오른쪽(+x)."""
    if not isinstance(curl_deg, dict):
        curl_deg = {f: curl_deg for f in hg.FINGERS}
    s = -1.0 if right else 1.0                       # 검지 쪽 x 부호
    lm = np.zeros((21, 3))
    xs = {"index": 0.03, "middle": 0.01, "ring": -0.01, "pinky": -0.03}
    for f, (mcp, pip, dip, tip) in hg.FINGERS.items():
        base = np.array([s * xs[f], 0.08, 0.0])
        lm[mcp] = base
        ang, p = 0.0, base.copy()
        for j in (pip, dip, tip):
            ang += math.radians(curl_deg[f])
            p = p + SEG * np.array([0.0, math.cos(ang), -math.sin(ang)])   # 아래(−z)로 말림
            lm[j] = p
    for k, j in enumerate((1, 2, 3, 4)):            # 엄지 (계산에 안 씀)
        lm[j] = np.array([s * (0.03 + 0.015 * k), 0.02 + 0.015 * k, 0.0])
    return lm


def test_open_hand_is_open():
    lm = make_hand(0.0)
    assert hg.grip_closure(lm) == pytest.approx(0.0, abs=0.05)
    assert hg.classify_gesture(lm) == hg.GESTURE_OPEN


def test_fist_is_fist():
    lm = make_hand(85.0)
    assert hg.grip_closure(lm) > 0.9
    assert hg.classify_gesture(lm) == hg.GESTURE_FIST


def test_half_grip_is_between():
    g = hg.grip_closure(make_hand(40.0))
    assert 0.3 < g < 0.8


def test_point_gesture_and_direction():
    lm = make_hand({"index": 0.0, "middle": 85.0, "ring": 85.0, "pinky": 85.0})
    assert hg.classify_gesture(lm) == hg.GESTURE_POINT
    np.testing.assert_allclose(hg.pointing_dir(lm), [0, 1, 0], atol=1e-6)


def test_angles_do_not_depend_on_rotation():
    """쥔 정도는 손을 어떻게 돌려도 같아야 한다 (world 좌표 축이 무엇이든 상관없다는 근거)."""
    lm = make_hand(50.0)
    a = 0.7
    R = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    assert hg.grip_closure(lm @ R.T) == pytest.approx(hg.grip_closure(lm), abs=1e-9)


@pytest.mark.parametrize("right", [True, False])
def test_palm_frame_is_orthonormal_and_faces_palm(right):
    origin, R = hg.palm_frame(make_hand(0.0, right), "right" if right else "left")
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-9)
    # y = 손목→중지 MCP. 가짜 손의 중지 MCP 가 손목에서 1 cm 옆이라 약 7° 기울어 있다 (실제 손도 비슷)
    assert np.dot(R[:, 1], [0, 1, 0]) > math.cos(math.radians(8))
    np.testing.assert_allclose(R[:, 2], [0, 0, -1], atol=1e-6)      # z = 손바닥이 보는 쪽 (아래)


def test_quaternion_roundtrip():
    _, R = hg.palm_frame(make_hand(0.0), "right")
    x, y, z, w = hg.rot_to_quat(R)
    Rq = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                   [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                   [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    np.testing.assert_allclose(Rq, R, atol=1e-9)


@pytest.mark.parametrize("roll_deg, expected", [(0, 0), (90, 90), (180, 180), (-90, 90)])
def test_wrist_rotation(roll_deg, expected):
    """손가락 방향(y)을 축으로 손을 굴림: 0 = 손바닥 아래, 90 = 옆, 180 = 위."""
    lm = make_hand(0.0)
    a = math.radians(roll_deg)
    Ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    _, R = hg.palm_frame(lm @ Ry.T, "right")
    assert hg.wrist_rotation_deg(R, np.array([0, 0, 1.0])) == pytest.approx(expected, abs=1e-6)


def test_distance_to_centerline_sign():
    pts = np.array([[0.70, 0, 0], [0.65, 0, 0], [0.80, 0, 0]])
    assert hg.distance_to_centerline(pts, 0.60) == pytest.approx(0.05)     # 환자 구역, 5 cm 남음
    assert hg.distance_to_centerline(pts - [0.10, 0, 0], 0.60) == pytest.approx(-0.05)   # 5 cm 침범


def test_gesture_filter_needs_consecutive_frames():
    f = GestureFilter(frames=5)
    outs = [f.update(True, hg.GESTURE_FIST) for _ in range(5)]
    assert [o[0] for o in outs[:4]] == [hg.GESTURE_NONE] * 4             # 4 프레임까지는 주먹 아님
    assert outs[4] == (hg.GESTURE_FIST, True)                             # 5 번째에 주먹 확정 + 정지 요청 1회
    assert f.update(True, hg.GESTURE_FIST) == (hg.GESTURE_FIST, False)    # 계속 쥐고 있어도 요청은 한 번


def test_gesture_filter_single_frame_glitch_does_not_stop():
    f = GestureFilter(frames=5)
    seq = [hg.GESTURE_OPEN] * 6 + [hg.GESTURE_FIST] + [hg.GESTURE_OPEN] * 3
    outs = [f.update(True, g) for g in seq]
    assert all(not e for _, e in outs)
    assert all(g != hg.GESTURE_FIST for g, _ in outs)


def test_gesture_filter_release_and_disable():
    f = GestureFilter(frames=3)
    for _ in range(3):
        f.update(True, hg.GESTURE_FIST)
    assert f.update(True, hg.GESTURE_OPEN)[0] == hg.GESTURE_NONE          # 손 펴면 주먹 바로 해제
    assert f.update(False, hg.GESTURE_FIST) == (hg.GESTURE_NONE, False)   # 손 없음
    off = GestureFilter(frames=3, fist_enabled=False)
    outs = [off.update(True, hg.GESTURE_FIST) for _ in range(5)]
    assert all(o == (hg.GESTURE_NONE, False) for o in outs)
