#!/usr/bin/env python3
"""/rally/robot/wrench 기록·요약 — 힘 노이즈(nudge 임계 근거)와 당김 힘(핸드오버 해제 임계 근거) 측정.

먼저: source scripts/env_robot_pc.sh && ros2 run rally_robot force_publisher   (다른 터미널)

    python3 scripts/measure_wrench.py --seconds 10                 # 정지 상태 노이즈: 축별 평균·표준편차·피크 → nudge 임계 제안
    python3 scripts/measure_wrench.py --seconds 10 --tag moving    # 로봇이 movel 로 움직이는 동안 (가속 구간 힘 피크 확인)
    python3 scripts/measure_wrench.py --pull 10                    # 당김 힘: 그리퍼로 막대를 잡은 채, Enter 마다 3 s 기록 ×10 → 피크 통계
    python3 scripts/measure_wrench.py --pull 10 --window 3 --tag rod_20N

결과 CSV 는 data/measure/ 에 남는다 (git 제외). 요약은 화면에 출력 — 그 숫자를 safety.yaml / skills.yaml 에 적는다.
"""
import argparse
import csv
import math
import os
import statistics
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped

AXES = ["fx", "fy", "fz", "tx", "ty", "tz"]


class Recorder(Node):
    def __init__(self):
        super().__init__("rally_measure_wrench")
        self.rows = []
        self.lock = threading.Lock()
        self.active = False
        self.count = 0
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self._cb, qos_profile_sensor_data)

    def _cb(self, m):
        self.count += 1
        if not self.active:
            return
        with self.lock:
            self.rows.append((time.time(), m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                              m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z))

    def record(self, seconds):
        with self.lock:
            self.rows = []
        self.active = True
        time.sleep(seconds)
        self.active = False
        with self.lock:
            return list(self.rows)


def summarize(rows, label):
    if len(rows) < 10:
        print(f"  [{label}] 샘플 {len(rows)}개 — 너무 적음. force_publisher 가 떠 있고 env 설정이 맞는지 확인")
        return None
    cols = list(zip(*rows))
    out = {}
    print(f"  [{label}] 샘플 {len(rows)}개, {len(rows)/(rows[-1][0]-rows[0][0]+1e-9):.0f} Hz")
    print("   축     평균      표준편차   최소      최대      |피크|")
    for name, col in zip(AXES, cols[1:]):
        mu, sd = statistics.fmean(col), statistics.pstdev(col)
        pk = max(abs(min(col)), abs(max(col)))
        out[name] = dict(mean=mu, std=sd, min=min(col), max=max(col), peak=pk)
        print(f"   {name:3s} {mu:8.3f}  {sd:8.3f}  {min(col):8.3f}  {max(col):8.3f}  {pk:8.3f}")
    fmag = [math.sqrt(r[1]**2 + r[2]**2 + r[3]**2) for r in rows]
    out["|F|"] = dict(mean=statistics.fmean(fmag), std=statistics.pstdev(fmag), peak=max(fmag))
    print(f"   |F| {out['|F|']['mean']:8.3f}  {out['|F|']['std']:8.3f}  {'':8s}  {'':8s}  {out['|F|']['peak']:8.3f}")
    # 샘플 간 변화율 (nudge 임계 근거 — wrench 50 Hz 면 20 ms)
    dfs = [abs(fmag[i] - fmag[i-1]) for i in range(1, len(fmag))]
    out["dF"] = dict(std=statistics.pstdev(dfs), peak=max(dfs))
    print(f"   한 샘플 |ΔF| 표준편차 {out['dF']['std']:.3f} N · 최대 {out['dF']['peak']:.3f} N")
    return out


def save(rows, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + AXES)
        w.writerows(rows)
    print(f"  저장 {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0, help="연속 기록 시간")
    ap.add_argument("--pull", type=int, default=0, help="당김 시험 횟수 (Enter 마다 --window 초 기록)")
    ap.add_argument("--window", type=float, default=3.0, help="당김 시험 1회 기록 시간")
    ap.add_argument("--tag", default="", help="파일 이름 꼬리표")
    opt = ap.parse_args()

    rclpy.init()
    node = Recorder()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def shutdown():
        ex.shutdown(timeout_sec=1.0)
        th.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    stamp = datetime.now().strftime("%m%d_%H%M%S")
    tag = f"_{opt.tag}" if opt.tag else ""
    time.sleep(1.0)
    if node.count == 0:
        print("/rally/robot/wrench 수신 없음 — force_publisher 와 env_robot_pc.sh 확인")
        shutdown()
        return 1
    try:
        if opt.pull:
            peaks = []
            print(f"당김 시험 {opt.pull}회. 그리퍼가 물체를 잡은 상태에서 Enter → {opt.window:.0f} s 동안 평소처럼 당기세요.")
            for i in range(1, opt.pull + 1):
                input(f"  [{i}/{opt.pull}] 준비되면 Enter ")
                rows = node.record(opt.window)
                s = summarize(rows, f"pull {i}")
                if s:
                    peaks.append(s["|F|"]["peak"])
                    save(rows, f"data/measure/pull{tag}_{stamp}_{i:02d}.csv")
            if peaks:
                print(f"\n당김 |F| 피크 {len(peaks)}회: 평균 {statistics.fmean(peaks):.2f} N · 최소 {min(peaks):.2f} · 최대 {max(peaks):.2f} N")
                print(f"→ Handover.release_force_threshold_n 후보: 최소 피크의 60~70 % ≈ {min(peaks)*0.65:.1f} N (노이즈 피크보다 충분히 커야 함)")
        else:
            print(f"{opt.seconds:.0f} s 기록 중… (정지 노이즈면 로봇·그리퍼를 건드리지 마세요)")
            rows = node.record(opt.seconds)
            s = summarize(rows, opt.tag or "still")
            if s:
                save(rows, f"data/measure/wrench{tag}_{stamp}.csv")
                print(f"\n→ safety.yaml 제안: nudge_dfdt_n ≥ {max(5 * s['dF']['std'], 2 * s['dF']['peak']):.1f} N "
                      f"(한 샘플 |ΔF| 표준편차 ×5 또는 최대 ×2 중 큰 값) · force_limit_n 25 N 은 |F| 피크 {s['|F|']['peak']:.1f} N 보다 충분히 큰지 확인")
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
