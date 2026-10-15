"""body_pose 관절 화면 그리기 (10/15) — 순수 함수, MediaPipe·카메라 없이."""
import math

import numpy as np
import pytest

pytest.importorskip("cv2")
from rally_interfaces.msg import BodyPose  # noqa: E402
from rally_human.body_pose_node import SKELETON, draw_debug  # noqa: E402


def _out(detected=True, comp="none"):
    o = BodyPose()
    o.detected, o.trunk_lean_deg, o.shoulder_elevation_deg, o.elbow_extension_deg = detected, 9.0, math.nan, 142.0
    o.compensation_type, o.compensation_detected = comp, comp != "none"
    return o


def test_draws_skeleton_and_keeps_input():
    img = np.zeros((480, 848, 3), np.uint8)
    pts = [(0.5, 0.5, 0.9)] * 33
    pts[11], pts[13] = (0.4, 0.4, 0.9), (0.4, 0.6, 0.9)
    outimg = draw_debug(img, pts, _out())
    assert outimg.shape == img.shape and img.sum() == 0          # 입력은 그대로
    assert outimg[int(0.5 * 480), int(0.4 * 848)].any()          # 어깨-팔꿈치 선 위 픽셀이 그려짐
    assert (11, 13) in SKELETON


def test_no_person_and_nan_values_do_not_fail():
    img = np.zeros((120, 160, 3), np.uint8)
    assert draw_debug(img, None, _out(detected=False)).shape == img.shape
    assert draw_debug(img, [(0.1, 0.1, 0.2)] * 5, _out(comp="trunk_lean")).shape == img.shape   # 점 수가 적어도
