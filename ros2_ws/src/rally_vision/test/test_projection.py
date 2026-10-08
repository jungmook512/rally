"""projection · segmenter 후처리 단위 테스트 — 카메라·로봇 없이. 가짜 깊이·마스크·변환으로 수식을 검증한다."""
import math

import numpy as np
import pytest

from rally_vision import projection as pj
from rally_vision.segmenter_node import SimpleTracks, parse_alias

K = [600.0, 0.0, 424.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]   # 848×480 RealSense 비슷한 값


def test_pixel_to_camera_center_and_offset():
    p = pj.pixel_to_camera(424, 240, 0.5, K)
    assert np.allclose(p, [0, 0, 0.5])
    p = pj.pixel_to_camera(424 + 600, 240, 1.0, K)       # 초점거리만큼 오른쪽 → x = z
    assert np.allclose(p, [1.0, 0.0, 1.0])


def test_pose_to_matrix_rotation_90deg_about_z():
    s = math.sin(math.pi / 4)
    T = pj.pose_to_matrix(1.0, 2.0, 3.0, 0.0, 0.0, s, s)   # z축 90°
    assert np.allclose(pj.transform_points(T, [[1.0, 0.0, 0.0]])[0], [1.0, 3.0, 3.0], atol=1e-9)


def test_base_from_tcp_converts_mm_to_m():
    g2c = np.eye(4); g2c[:3, 3] = [33.0, 75.0, -233.0]      # 선생님 npy 와 같은 단위 (mm)
    tcp = pj.pose_to_matrix(0.4, 0.0, 0.3, 0, 0, 0, 1)
    T = pj.base_from_tcp(tcp, g2c)
    assert np.allclose(T[:3, 3], [0.433, 0.075, 0.067])
    assert np.allclose(g2c[:3, 3], [33.0, 75.0, -233.0])    # 원본은 안 바뀜


def test_mask_depth_median_ignores_zero_and_background():
    depth = np.zeros((40, 40), dtype=np.uint16)
    depth[:, :] = 900                                      # 배경 0.9 m
    depth[10:30, 10:30] = 500                              # 물체 0.5 m
    depth[12, 12] = 0                                      # 구멍
    mask = np.zeros((40, 40), bool); mask[10:30, 10:30] = True
    assert pj.mask_depth_median(depth, mask) == pytest.approx(0.5)
    assert pj.mask_depth_median(depth, np.zeros((40, 40), bool)) is None


def test_principal_axis_of_horizontal_bar():
    mask = np.zeros((100, 200), bool); mask[45:55, 20:180] = True
    cx, cy, (du, dv), ratio = pj.mask_principal_axis(mask)
    assert (cx, cy) == pytest.approx((99.5, 49.5))
    assert abs(du) > 0.99 and ratio > 5                     # 가로 막대 → 주축 가로, 길쭉함
    circle = np.zeros((100, 100), bool)
    yy, xx = np.mgrid[:100, :100]; circle[(xx - 50) ** 2 + (yy - 50) ** 2 < 400] = True
    assert pj.mask_principal_axis(circle)[3] < 1.1          # 원 → 비 ≈ 1 (yaw 무의미)


def test_yaw_in_frame_identity_and_rotated():
    bar = (100.0, 100.0, (1.0, 0.0), 10.0)                 # 영상 가로 방향 주축
    p0 = pj.pixel_to_camera(100, 100, 0.5, K)
    assert pj.yaw_in_frame(np.eye(4), p0, bar, 0.5, K) == pytest.approx(0.0, abs=1e-6)
    s = math.sin(math.pi / 4)
    Tz90 = pj.pose_to_matrix(0, 0, 0, 0, 0, s, s)
    assert pj.yaw_in_frame(Tz90, p0, bar, 0.5, K) == pytest.approx(math.pi / 2, abs=1e-6)


def test_simple_tracks_velocity_and_moving():
    tr = SimpleTracks(moving_mps=0.05)
    a = dict(class_name="ball", center=np.array([0.3, 0.0, 0.02]))
    tr.update([a], 0.0)
    assert a["track_id"] == 1 and not a["is_moving"]
    b = dict(class_name="ball", center=np.array([0.32, 0.0, 0.02]))   # 0.1 s 에 2 cm → 0.2 m/s
    tr.update([b], 0.1)
    assert b["track_id"] == 1 and b["is_moving"] and b["velocity"][0] == pytest.approx(0.1, abs=1e-6)  # 0.5·0 + 0.5·0.2
    c = dict(class_name="cup", center=np.array([0.32, 0.0, 0.02]))     # 다른 클래스 → 새 ID
    tr.update([c], 0.2)
    assert c["track_id"] == 2


def test_parse_alias():
    assert parse_alias("sports ball:ball, cup:cup") == {"sports ball": "ball", "cup": "cup"}


# ---------------------------------------------------------------- hand_eye.yaml · camera_tf
def test_load_hand_eye_roundtrip(tmp_path):
    import yaml
    from rally_vision import projection as pj
    p = tmp_path / "hand_eye.yaml"
    yaml.safe_dump({"tcp_T_camera_optical": {"xyz_m": [0.03728, 0.08087, -0.205],
                                             "quat_xyzw": [-0.005884, -0.004204, 0.999924, -0.009944]},
                    "depth_scale": 0.971, "tcp_offset_from_flange_m": [0, 0, 0.228]}, open(p, "w"))
    he = pj.load_hand_eye(str(p))
    assert abs(he["depth_scale"] - 0.971) < 1e-9
    T = he["T_tcp_cam_mm"]
    assert T.shape == (4, 4)
    assert np.allclose(T[:3, 3], [37.28, 80.87, -205.0], atol=1e-6)          # m → mm
    assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-5)          # 회전 행렬
    assert T[0, 0] < -0.99 and T[2, 2] > 0.99                                  # 카메라 z 는 TCP z 와 같은 방향(아래), x·y 는 반대
    assert np.allclose(he["tcp_offset_m"], [0, 0, 0.228])


def test_load_hand_eye_empty(tmp_path):
    from rally_vision import projection as pj
    p = tmp_path / "hand_eye.yaml"
    p.write_text("# 비어 있음\n")
    he = pj.load_hand_eye(str(p))
    assert he["T_tcp_cam_mm"] is None and he["depth_scale"] == 1.0
    assert pj.load_hand_eye("")["T_tcp_cam_mm"] is None


def test_rot_to_quat_matches_quat_to_rot():
    from rally_vision import projection as pj
    from rally_vision.camera_tf_node import _rot_to_quat
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        Rm = pj.quat_to_rot(*q)
        q2 = np.array(_rot_to_quat(Rm))
        assert np.allclose(Rm, pj.quat_to_rot(*q2), atol=1e-9)   # 부호 모호성은 행렬로 비교


# ---------------------------------------------------------------- transform_source 선택 (PR #73 리뷰)
def test_pick_transform_source_matrix():
    from rally_vision.segmenter_node import pick_transform_source as pick
    # (source, tf_ok, tcp_ok) → 기대
    cases = {
        ("tcp_npy", True, True): "tcp_npy", ("tcp_npy", True, False): None, ("tcp_npy", False, True): "tcp_npy",
        ("tf", True, True): "tf", ("tf", False, True): None, ("tf", False, False): None,
        ("auto", True, True): "tf", ("auto", False, True): "tcp_npy", ("auto", True, False): "tf", ("auto", False, False): None,
    }
    for (src, tf_ok, tcp_ok), want in cases.items():
        assert pick(src, tf_ok, tcp_ok) == want, (src, tf_ok, tcp_ok)
