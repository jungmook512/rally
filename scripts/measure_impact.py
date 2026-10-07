#!/usr/bin/env python3
"""공을 패드에 던졌을 때 로봇에 걸리는 충격 힘 측정 (모드 3 검토용). 로봇은 움직이지 않는다 — 패드를 쥔 채 서 있기만 하면 된다.

준비: 로봇이 벨크로 패드를 쥐고 대기 자세(패드 면이 환자 쪽)로 서 있고, force_publisher 가 떠 있을 것.
      safety_monitor 가 떠 있으면 충격이 nudge 로 잡혀 STOP 이 난다 → 이 스크립트가 /rally/safety/contact_expected 를 5 Hz 로 올려
      nudge 만 끈다. 절대 힘 한계(25 N)는 그대로 살아 있다 (공이 25 N 을 넘기면 STOP — 그것도 측정 결과다).

    source scripts/env_robot_pc.sh && ros2 run rally_robot force_publisher     (다른 터미널)
    python3 scripts/measure_impact.py --throws 10                 # 10회 던지기, 충격 자동 검출
    python3 scripts/measure_impact.py --throws 10 --threshold 2   # 살살 토스라 피크가 작으면 임계 낮추기
    python3 scripts/measure_impact.py --throws 5 --no-mask        # safety_monitor 가 멈추는지 보고 싶을 때 (nudge 마스킹 끔)
    python3 scripts/measure_impact.py --throws 5 --mark           # 임계 없이: 던지기 직전 Enter → 그 뒤 3 s 창의 최대 |ΔF|·샘플간 ΔF 를 무조건 보고 (작은 충격 정량화)

출력: 던질 때마다 피크 |ΔF|, 축별 피크(패드 법선 방향 확인), 접촉 시간, 한 주기(10 ms) 최대 ΔF(nudge 설계용), 요약 + CSV(data/measure/).
"""
import argparse
import csv
import math
import os
import statistics
import sys
import threading
import time
from collections import deque
from datetime import datetime

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool

AXES = ["fx", "fy", "fz"]


class Impact(Node):
    def __init__(self, mask_nudge: bool):
        super().__init__("rally_measure_impact")
        self.rows = []                       # (t, fx, fy, fz, tx, ty, tz) 전체 기록
        self.buf = deque(maxlen=100)         # 최근 1 s (영점 추정용)
        self.lock = threading.Lock()
        self.count = 0
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self._cb, qos_profile_sensor_data)
        self.mask = mask_nudge
        if mask_nudge:
            self.pub_contact = self.create_publisher(Bool, "/rally/safety/contact_expected",
                                                     QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))
            self.create_timer(0.2, lambda: self.pub_contact.publish(Bool(data=True)))

    def _cb(self, m):
        self.count += 1
        row = (time.time(), m.wrench.force.x, m.wrench.force.y, m.wrench.force.z, m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z)
        with self.lock:
            self.rows.append(row)
            self.buf.append(row)

    def baseline(self):
        with self.lock:
            b = list(self.buf)
        if len(b) < 20:
            return None
        return [statistics.median(r[i] for r in b) for i in (1, 2, 3)]

    def stop_mask(self):
        if self.mask:
            self.pub_contact.publish(Bool(data=False))


def detect_hit(node, base, threshold, timeout_s):
    """영점 대비 |ΔF| 가 threshold 를 넘는 순간부터 다시 threshold/2 아래로 0.2 s 내려올 때까지를 한 번의 충격으로 본다."""
    t0 = time.monotonic()
    start_idx = None
    with node.lock:
        start_len = len(node.rows)
    while time.monotonic() - t0 < timeout_s:
        time.sleep(0.01)
        with node.lock:
            rows = node.rows[start_len:]
        if not rows:
            continue
        devs = [math.sqrt(sum((r[i + 1] - base[i]) ** 2 for i in range(3))) for r in rows]
        if start_idx is None:
            for k, d in enumerate(devs):
                if d >= threshold:
                    start_idx = k
                    break
            continue
        # 충격 시작 후: threshold/2 아래로 0.2 s(20 샘플) 유지되면 끝
        tail = devs[-20:]
        if len(tail) == 20 and max(tail) < threshold / 2:
            seg = rows[start_idx:len(rows) - 20]
            seg_devs = devs[start_idx:len(devs) - 20]
            if not seg:
                return None
            peak_i = max(range(len(seg_devs)), key=lambda i: seg_devs[i])
            axis_peak = [max(abs(r[i + 1] - base[i]) for r in seg) for i in range(3)]
            pre = devs[max(start_idx - 1, 0):len(devs) - 20]              # 영점 → 첫 충격 샘플의 점프까지 포함 (nudge 가 보는 값)
            dfs = [abs(pre[i] - pre[i - 1]) for i in range(1, len(pre))] or [0.0]
            return dict(peak=seg_devs[peak_i], axis=axis_peak, duration_s=seg[-1][0] - seg[0][0],
                        dfdt_max=max(dfs), t=seg[peak_i][0], samples=len(seg))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--throws", type=int, default=10, help="던지기 횟수")
    ap.add_argument("--threshold", type=float, default=3.0, help="충격 검출 임계 |ΔF| [N] (영점 대비)")
    ap.add_argument("--timeout", type=float, default=30.0, help="한 번 던지기 대기 시간 [s]")
    ap.add_argument("--no-mask", action="store_true", help="contact_expected 하트비트 끄기 (safety_monitor nudge 반응 확인용)")
    ap.add_argument("--mark", action="store_true", help="던지기 직전 Enter 로 표시 → 이후 --window 초 창의 최대값을 임계 없이 보고")
    ap.add_argument("--window", type=float, default=3.0, help="--mark 분석 창 [s]")
    ap.add_argument("--tag", default="pad")
    opt = ap.parse_args()

    rclpy.init()
    node = Impact(mask_nudge=not opt.no_mask)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def shutdown():
        node.stop_mask()
        ex.shutdown(timeout_sec=1.0)
        th.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    time.sleep(1.5)
    if node.count == 0:
        print("/rally/robot/wrench 수신 없음 — force_publisher 와 env_robot_pc.sh 확인")
        shutdown()
        return 1
    print(f"충격 측정 {opt.throws}회 · 임계 {opt.threshold:.1f} N · nudge 마스킹 {'OFF' if opt.no_mask else 'ON(contact_expected)'}")
    print("로봇은 패드를 쥔 채 가만히. 각 회차: 영점 잡힌 뒤 '던지세요' 가 뜨면 던진다. 패드 외 로봇 몸체는 맞히지 말 것.")
    hits = []
    stamp = datetime.now().strftime("%m%d_%H%M%S")
    try:
        for i in range(1, opt.throws + 1):
            time.sleep(1.0)                                   # 직전 충격 여운
            base = node.baseline()
            while base is None:
                time.sleep(0.2)
                base = node.baseline()
            if opt.mark:
                input(f"\n[{i}/{opt.throws}] 준비되면 Enter 를 누르고 바로 던지세요 (이후 {opt.window:.0f} s 분석) ")
                base = node.baseline()
                with node.lock:
                    start_len = len(node.rows)
                time.sleep(opt.window)
                with node.lock:
                    rows = node.rows[start_len:]
                if len(rows) < 5:
                    print("   샘플 없음"); continue
                devs = [math.sqrt(sum((r[k + 1] - base[k]) ** 2 for k in range(3))) for r in rows]
                fm = [math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2) for r in rows]
                dfs = [abs(fm[k] - fm[k - 1]) for k in range(1, len(fm))] or [0.0]
                pk = max(range(len(devs)), key=lambda k: devs[k])
                above = [k for k, d in enumerate(devs) if d >= devs[pk] * 0.5]
                h = dict(peak=devs[pk], axis=[abs(rows[pk][k + 1] - base[k]) for k in range(3)],
                         duration_s=rows[above[-1]][0] - rows[above[0]][0], dfdt_max=max(dfs), t=rows[pk][0], samples=len(rows))
                hits.append(h)
                ax = max(range(3), key=lambda k: h["axis"][k])
                print(f"   창 {len(rows)} 샘플({len(rows)/opt.window:.0f} Hz) · 최대 |ΔF| {h['peak']:.2f} N @ +{rows[pk][0]-rows[0][0]:.2f} s · 축별 {[round(a, 2) for a in h['axis']]} (주축 {AXES[ax]}) · "
                      f"반치폭 {h['duration_s']*1000:.0f} ms · 샘플간 ΔF 최대 {h['dfdt_max']:.2f} N")
                continue
            print(f"\n[{i}/{opt.throws}] 영점 {[round(b, 2) for b in base]} N — 던지세요 (최대 {opt.timeout:.0f} s 대기)")
            h = detect_hit(node, base, opt.threshold, opt.timeout)
            if h is None:
                print("   충격 감지 없음 (임계 아래거나 시간 초과) — --threshold 를 낮추거나 다시")
                continue
            hits.append(h)
            ax = max(range(3), key=lambda k: h["axis"][k])
            print(f"   피크 |ΔF| {h['peak']:.2f} N · 축별 {[round(a, 2) for a in h['axis']]} (주축 {AXES[ax]}) · 접촉 {h['duration_s']*1000:.0f} ms · "
                  f"10 ms ΔF 최대 {h['dfdt_max']:.2f} N · 샘플 {h['samples']}")
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        os.makedirs("data/measure", exist_ok=True)
        path = f"data/measure/impact_{opt.tag}_{stamp}.csv"
        with node.lock:
            rows = list(node.rows)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "fx", "fy", "fz", "tx", "ty", "tz"])
            w.writerows(rows)
        print(f"\n원본 저장 {path} ({len(rows)} 샘플)")
        if hits:
            peaks = [h["peak"] for h in hits]
            dfs = [h["dfdt_max"] for h in hits]
            durs = [h["duration_s"] * 1000 for h in hits]
            print(f"충격 {len(hits)}회: 피크 평균 {statistics.fmean(peaks):.2f} N · 최소 {min(peaks):.2f} · 최대 {max(peaks):.2f} N")
            print(f"  접촉 시간 평균 {statistics.fmean(durs):.0f} ms · 10 ms ΔF 최대 {max(dfs):.2f} N")
            print(f"→ 판단 근거: 힘 한계 25 N 대비 최대 피크 {max(peaks):.1f} N / nudge 임계(현재 5 N) 대비 ΔF 최대 {max(dfs):.1f} N "
                  f"→ 모드 3 받기 구간은 contact_expected 로 nudge 를 꺼야 {'함' if max(dfs) >= 5.0 else '안전 (그래도 끄는 편이 안전)'}")
            print(f"→ CatchBall '받음' 판정 힘 임계 후보: 최소 피크의 60 % ≈ {min(peaks) * 0.6:.1f} N (비전 판정과 함께 쓸 보조값)")
        shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
