"""cameras.yaml → RealSense launch 인자·노출 파라미터 (로봇 없이)."""
from pathlib import Path

from rally_bringup import camera_params as cp

CFG = cp.load(Path(__file__).resolve().parents[1] / "config" / "cameras.yaml")


def test_two_cameras_60fps_with_serials():
    wrist, side = CFG["wrist"], CFG["side"]
    assert isinstance(wrist["serial"], str) and isinstance(side["serial"], str)      # 숫자로 읽히면 앞자리 0 등이 깨진다
    assert wrist["serial"] != side["serial"]
    for cam in (wrist, side):
        assert cam["color_profile"].endswith(",60")
    assert wrist["depth_profile"].endswith(",60")


def test_launch_args():
    w = cp.rs_launch_args(CFG["wrist"], depth=True)
    assert w["serial_no"] == "_140122076335" and w["camera_namespace"] == "" and w["camera_name"] == "camera"
    assert w["align_depth.enable"] == "true" and w["depth_module.depth_profile"] == "848,480,60"
    s = cp.rs_launch_args(CFG["side"], depth=False)
    assert s["serial_no"] == "_147122075430" and (s["camera_namespace"], s["camera_name"]) == ("side", "side")
    assert s["enable_depth"] == "false" and "align_depth.enable" not in s
    assert CFG["side"]["node"] == "/side/side"                 # 토픽 /side/side/color/... 와 같은 이름 (INTERFACES 2.1)


def test_exposure_params_order():
    fixed = cp.exposure_params(True, CFG)
    assert fixed == [("rgb_camera.enable_auto_exposure", False), ("rgb_camera.exposure", 20), ("rgb_camera.gain", 128)]
    auto = cp.exposure_params(False, CFG)
    assert auto == [("rgb_camera.gain", cp.AUTO_GAIN), ("rgb_camera.enable_auto_exposure", True)]   # gain 먼저 되돌림
