#!/usr/bin/env python3
"""손 따라가기(모드 5 RECEIVE servol) 준비 측정 — HandState 지연 · 손바닥 위치 떨림 · 발행 간격.

hand_pose 노드가 켜진 상태에서 실행. 지금 손 거리가 계속 표시되고, 원하는 거리에 손을 두고 **Enter** 를 누르면
--seconds 동안 잰다 (손은 가만히). --auto 면 예전처럼 3초 뒤 자동 시작.
결과는 터미널과 data/measure/hand_jitter_MMDD.txt 에 (실행마다 이어 붙임).
    python3 scripts/measure_hand_jitter.py                      # 10 s
    python3 scripts/measure_hand_jitter.py --seconds 20 --label 50cm
결과
- 지연: 받은 시각 − header.stamp(카메라 프레임 시각). 카메라 드라이버 + 손 노드 처리 시간 합 (같은 PC 시계 기준)
- 떨림: 손바닥 위치 x·y·z 표준편차 [mm], 손목 각도 표준편차 [°] — 정정인 servol 필터(One-Euro 등) 값 정할 때 씀
- 발행: 평균 Hz, 손 검출 비율, 가장 긴 끊김 (손 놓침·프레임 누락)
좌표: tf 가 있으면 base_link, 없으면 카메라 좌표 (frame_id 로 표시). 떨림 크기는 어느 좌표든 비슷하다.
손목 각도는 hand_pose 의 손목 필터(0.5 s 중간값) 뒤 값이다 — 원래 흔들림이 아니라 내보내는 값의 흔들림.
print: Enter 를 기다리는 동안 지금 손 거리를 한 줄에 덮어써 보여 주려고 씀 (로거는 줄을 덮어쓸 수 없음). 결과는 로거로.
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from rally_interfaces.msg import HandState


class Meter(Node):
    def __init__(self) -> None:
        super().__init__("rally_measure_hand_jitter")
        self.rows: list[tuple] = []          # (받은 시각 s, 지연 ms, 검출, x, y, z, 손목, frame)
        self.on = False
        self.last: HandState | None = None
        self.create_subscription(HandState, "/rally/perception/hand", self._on, qos_profile_sensor_data)

    def _on(self, m: HandState) -> None:
        self.last = m
        if not self.on:
            return
        now = self.get_clock().now()
        lat_ms = (now - Time.from_msg(m.header.stamp)).nanoseconds * 1e-6
        p = m.palm_pose.position
        self.rows.append((time.monotonic(), lat_ms, m.detected, p.x, p.y, p.z, m.wrist_angle_deg, m.header.frame_id))


def summarize(rows: list[tuple], seconds: float) -> str:
    if not rows:
        return "HandState 를 못 받음 — hand_pose 노드가 켜져 있는지, 같은 ROS 설정(도메인·RMW)인지"
    t = np.array([r[0] for r in rows])
    lat = np.array([r[1] for r in rows])
    det = np.array([r[2] for r in rows])
    gaps = np.diff(t) * 1000 if len(t) > 1 else np.array([0.0])
    lines = [f"메시지 {len(rows)}개 / {seconds:.0f} s → {len(rows) / seconds:.1f} Hz, 손 검출 {det.mean() * 100:.0f} %, "
             f"가장 긴 간격 {gaps.max():.0f} ms (평균 {gaps.mean():.0f} ms)",
             f"지연(카메라 프레임 → 손 메시지): 평균 {lat.mean():.0f} ms · 중간값 {np.median(lat):.0f} ms · 95% {np.percentile(lat, 95):.0f} ms"]
    d = [r for r in rows if r[2]]
    # 카메라 좌표는 깊이가 없으면 z = 0 → 뺀다. base_link 는 z 가 테이블 높이라 0 근처도 정상
    pos = np.array([r[3:6] for r in d if np.isfinite(r[5]) and (r[5] > 0 or not r[7].startswith("camera"))])
    if len(pos) >= 10:
        std = pos.std(axis=0) * 1000
        step = np.linalg.norm(np.diff(pos, axis=0), axis=1) * 1000
        where = "높이 z" if d[-1][7] == "base_link" else "카메라 거리"
        lines.append(f"손바닥 위치 떨림 ({d[-1][7]}, {where} 약 {np.median(pos[:, 2]) * 100:.0f} cm, {len(pos)} 프레임): "
                     f"표준편차 x {std[0]:.1f} · y {std[1]:.1f} · z {std[2]:.1f} mm, "
                     f"프레임 간 이동 중간값 {np.median(step):.1f} mm · 95% {np.percentile(step, 95):.1f} mm")
    else:
        lines.append("손바닥 위치 없음 (깊이 없음 — RealSense 정렬 깊이가 켜져 있는지) 또는 손 검출 부족")
    w = np.array([r[6] for r in d if np.isfinite(r[6])])
    if len(w) >= 10:
        lines.append(f"손목 각도: 평균 {w.mean():.0f}°, 표준편차 {w.std():.1f}°, 범위 {w.min():.0f}~{w.max():.0f}°")
    return "\n  ".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--label", default="", help="결과 앞에 붙일 이름 (예: 50cm)")
    ap.add_argument("--auto", action="store_true", help="Enter 없이 3초 뒤 자동 시작")
    a = ap.parse_args()
    rclpy.init()
    node = Meter()
    if a.auto:
        print("손을 카메라 앞에 가만히 두세요. 3초 뒤 시작", flush=True)
        end_wait = time.monotonic() + 3.0
        while time.monotonic() < end_wait:
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        import select
        import sys
        print("손을 원하는 거리에 두고 Enter 를 누르면 시작 (지금 거리가 아래에 계속 표시됨)", flush=True)
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            m = node.last
            if m is None:
                state = "HandState 없음 — hand_pose 노드 확인"
            elif not m.detected:
                state = "손 없음"
            elif m.header.frame_id.startswith("camera") and m.palm_pose.position.z > 0:
                state = f"카메라 {m.palm_pose.position.z * 100:5.1f} cm"
            elif m.header.frame_id == "base_link":
                state = f"base_link x {m.palm_pose.position.x:.3f} m (tf 있음 — 카메라 거리 아님)"
            else:
                state = "깊이 없음"
            print(f"\r  {state:<60}", end="", flush=True)
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                print()
                break
    node.on = True
    t0 = time.monotonic()
    print(f"측정 중 {a.seconds:.0f} s ...", flush=True)
    while time.monotonic() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.on = False
    text = f"결과{' [' + a.label + ']' if a.label else ''}\n  {summarize(node.rows, a.seconds)}"
    node.get_logger().info(text)
    # 결과를 파일에도 쌓는다 (data/ 는 git 제외)
    from datetime import datetime
    from pathlib import Path
    out = Path(__file__).resolve().parents[1] / "data" / "measure" / f"hand_jitter_{datetime.now():%m%d}.txt"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("a") as f:
        f.write(f"[{datetime.now():%H:%M:%S}] {text}\n\n")
    print(f"저장: {out}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
