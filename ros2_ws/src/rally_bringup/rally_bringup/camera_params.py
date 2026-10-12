"""rally_bringup/rally_bringup/camera_params.py — RealSense 두 대 설정 (config/cameras.yaml)
담당: 조정묵

  load()                     cameras.yaml
  rs_launch_args(cam, ...)   realsense2_camera rs_launch.py 인자 (시리얼·해상도·fps)
  exposure_params(fixed, cfg) 노출 고정(모드 3) / 자동(평소) 으로 바꿀 파라미터 (이름, 값) 순서대로
  set_camera_params (실행)   카메라 노드가 뜰 때까지 기다렸다가 노출을 한 번 넣고 끝 — 측면 카메라용 (launch 가 띄움)
                              ros2 run rally_bringup set_camera_params --node /side/side --fixed

노출은 드라이버가 실행 중에 바꿀 수 있는 값이라(재시작 없음) brain 이 모드 3 에 들어갈 때·나올 때 손목 카메라를 바꾼다
(rally_brain/camera_exposure.py). fps·해상도는 켤 때 정하는 값이라 항상 60 fps 로 띄운다.
launch 인자로 노출을 넘기면 드라이버가 무시하는 경우가 있어(10/10 실측, emitter 도 같음) 뜬 뒤에 파라미터로 넣는다.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

import yaml

CONFIG = "cameras.yaml"
AUTO_GAIN = 64          # 자동 노출일 때 드라이버 기본 gain (10/10 실측). 고정 128 에서 돌아올 때 먼저 되돌린다


def load(path: str | Path | None = None) -> dict[str, Any]:
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("rally_bringup")) / "config" / CONFIG
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def rs_launch_args(cam: dict[str, Any], depth: bool) -> dict[str, str]:
    """시리얼은 앞에 _ — 숫자만인 문자열을 ROS 가 정수로 바꾸는 것을 피하는 드라이버 관례 (realsense-ros README)."""
    args = {
        "serial_no": f"_{cam['serial']}",
        "camera_namespace": str(cam.get("namespace", "")),
        "camera_name": str(cam["name"]),
        "rgb_camera.color_profile": str(cam["color_profile"]),
        "enable_depth": "true" if depth else "false",
    }
    if depth:
        args["depth_module.depth_profile"] = str(cam.get("depth_profile", cam["color_profile"]))
        args["align_depth.enable"] = "true"
    return args


def exposure_params(fixed: bool, cfg: dict[str, Any]) -> list[tuple[str, Any]]:
    """순서대로 넣는다. 고정: 자동 끄기 → 노출 → gain. 자동: gain 원래대로 → 자동 켜기."""
    if fixed:
        fx = cfg.get("fixed_exposure") or {}
        return [("rgb_camera.enable_auto_exposure", False),
                ("rgb_camera.exposure", int(fx.get("exposure", 20))),
                ("rgb_camera.gain", int(fx.get("gain", 128)))]
    return [("rgb_camera.gain", AUTO_GAIN), ("rgb_camera.enable_auto_exposure", True)]


def main(argv: list[str] | None = None) -> int:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient

    ap = argparse.ArgumentParser(description="RealSense 노출 한 번 설정 (카메라가 뜰 때까지 기다림)")
    ap.add_argument("--node", required=True, help="드라이버 노드 (예: /side/side)")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--fixed", action="store_true")
    mode.add_argument("--auto", action="store_true")
    ap.add_argument("--wait_s", type=float, default=120.0, help="카메라를 기다리는 최대 시간")
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = rclpy.create_node("rally_set_camera_params")
    log = node.get_logger()
    try:
        client = AsyncParameterClient(node, args.node)
        if not client.wait_for_services(timeout_sec=args.wait_s):
            log.warn(f"{args.node} 를 {args.wait_s:.0f} s 동안 못 찾음 — 노출 설정 안 함 (카메라가 안 꽂혔거나 시리얼이 다름)")
            return 1
        time.sleep(2.0)   # 드라이버가 스트림을 시작한 뒤에 넣어야 먹는다
        params = [Parameter(name, value=value) for name, value in exposure_params(args.fixed, load())]
        future = client.set_parameters(params)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        results = future.result().results if future.done() and future.result() else []
        bad = [p.name for p, r in zip(params, results) if not r.successful]
        if len(results) != len(params) or bad:
            log.error(f"{args.node} 노출 설정 실패: {bad or '응답 없음'}")
            return 1
        log.info(f"{args.node} 노출 {'고정' if args.fixed else '자동'}: " + ", ".join(f"{p.name}={p.value}" for p in params))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
