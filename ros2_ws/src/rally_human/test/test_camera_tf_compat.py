"""#77 리뷰 대응 — 영상 시각 tf 대체 조회, hand_eye.yaml depth_scale 읽기."""
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer

from rally_human.hand_pose_node import checked_depth_scale, load_depth_scale, lookup_camera_tf


def _buf_with_tf_at(sec: float) -> Buffer:
    b = Buffer()
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = "base_link", "camera_color_optical_frame"
    t.header.stamp = Time(nanoseconds=int(sec * 1e9)).to_msg()
    t.transform.rotation.w = 1.0
    b.set_transform(t, "test")
    return b


def test_exact_time_is_used_without_fallback():
    tf, fb = lookup_camera_tf(_buf_with_tf_at(10.0), "base_link", "camera_color_optical_frame",
                              Time(nanoseconds=int(10.0e9)), 0.15)
    assert tf is not None and not fb


def test_newer_image_uses_latest_tf_within_age():
    """camera_tf 10 Hz: 영상이 마지막 tf 보다 80 ms 최신 → 대신 씀."""
    tf, fb = lookup_camera_tf(_buf_with_tf_at(10.0), "base_link", "camera_color_optical_frame",
                              Time(nanoseconds=int(10.08e9)), 0.15)
    assert tf is not None and fb


def test_too_old_tf_is_rejected():
    tf, fb = lookup_camera_tf(_buf_with_tf_at(10.0), "base_link", "camera_color_optical_frame",
                              Time(nanoseconds=int(10.5e9)), 0.15)
    assert tf is None and not fb


def test_no_tf_at_all():
    tf, fb = lookup_camera_tf(Buffer(), "base_link", "camera_color_optical_frame", Time(nanoseconds=1), 0.15)
    assert tf is None


def test_depth_scale_from_yaml(tmp_path):
    p = tmp_path / "hand_eye.yaml"
    p.write_text("depth_scale: 0.971\n")
    assert load_depth_scale(str(p)) == (0.971, "")
    assert load_depth_scale("") == (1.0, "")                       # 설치 안 됨 → 조용히 1.0


def test_depth_scale_warns_when_file_unusable(tmp_path):
    """파일은 있는데 못 쓰면 1.0 + 경고 문구 (#91 리뷰 2)."""
    missing = tmp_path / "none.yaml"
    assert load_depth_scale(str(missing))[0] == 1.0 and load_depth_scale(str(missing))[1]
    nokey = tmp_path / "a.yaml"
    nokey.write_text("tcp_T_camera_optical: {}\n")
    assert load_depth_scale(str(nokey))[0] == 1.0 and "depth_scale" in load_depth_scale(str(nokey))[1]
    broken = tmp_path / "b.yaml"
    broken.write_text("depth_scale: [\n")
    assert load_depth_scale(str(broken))[0] == 1.0 and load_depth_scale(str(broken))[1]


def test_depth_scale_out_of_range_falls_back():
    """손 거리가 안전 판정에 바로 쓰이므로 이상한 배율은 1.0 (#91 리뷰 1)."""
    assert checked_depth_scale(0.971) == (0.971, "")
    for bad in (0.0971, 9.71, float("nan"), -1.0):
        v, why = checked_depth_scale(bad)
        assert v == 1.0 and why
