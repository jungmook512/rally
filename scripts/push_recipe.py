#!/usr/bin/env python3
"""모드 1 밀기 레시피 실험 — "밀기 속도 → 공 정지 거리" 를 레인별로 재고 식을 맞춘다 (todo: 밀기 파라미터 사전 실험 20회).

물리: 구르는 공은 거의 일정한 감속도 a 로 멈춘다 → 정지 거리 d = v² / (2a) + d0.
      레인마다 a(마찰)가 다르므로 레인별로 맞춘다. 결과 a, d0 로 목표 거리 → 밀기 속도: v = sqrt(2a (d - d0)).

준비: skill_server · safety_monitor · gripper_node(시뮬이면 scripts/sim_gripper.py) 가 떠 있을 것.
      정지 위치는 비전(/rally/perception/objects 의 ball) 이 있으면 자동, 없으면 --manual 로 줄자 값(cm)을 입력.

    python3 scripts/push_recipe.py --lane 1 --speeds 0.08 0.12 0.16 0.20 --repeats 3            # 비전 자동
    python3 scripts/push_recipe.py --lane 1 --speeds 0.08 0.12 0.16 --repeats 3 --manual        # 줄자
    python3 scripts/push_recipe.py --fit data/measure/push_recipe_lane1_1007_150000.csv          # 기록만 다시 맞추기

매 시도 전에 공을 레인 입구(skills.yaml mode1.lanes[lane].start)에 놓고 Enter. 정지 거리는 레인 입구에서 레인 방향으로 잰 값.
결과 CSV 는 data/measure/ (git 제외). 마지막에 a·d0 와 목표 거리별 추천 속도 표를 출력 → skills.yaml 에 옮긴다.
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
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKILLS_YAML = ROOT / "ros2_ws/src/rally_bringup/config/skills.yaml"
FIELDS = ["t", "lane", "speed_mps", "stop_x", "stop_y", "dist_m", "lateral_m", "pred_x", "pred_y", "pred_err_m", "source"]


def load_lane(lane):
    import yaml
    cfg = yaml.safe_load(SKILLS_YAML.read_text(encoding="utf-8"))
    lanes = cfg["mode1"]["lanes"]
    ln = lanes[lane]
    return float(ln["start"][0]), float(ln["start"][1]), float(ln.get("dir_deg", 0.0)), ln.get("name", str(lane))


def fit(rows):
    """d = v²/(2a) + d0 최소자승. rows: [(v, d)] → (a, d0, rmse)."""
    xs = [v * v for v, _ in rows]
    ys = [d for _, d in rows]
    n = len(rows)
    if n < 2 or len(set(xs)) < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx          # k = 1/(2a)
    d0 = my - k * mx
    if k <= 0:
        return None
    rmse = math.sqrt(sum((k * x + d0 - y) ** 2 for x, y in zip(xs, ys)) / n)
    return 1.0 / (2.0 * k), d0, rmse


def report(rows_all):
    by_v = {}
    for r in rows_all:
        by_v.setdefault(round(float(r["speed_mps"]), 3), []).append(float(r["dist_m"]))
    print("\n속도별 정지 거리")
    print("  속도[m/s]  횟수   평균[m]  표준편차[m]")
    for v in sorted(by_v):
        ds = by_v[v]
        print(f"  {v:8.3f}  {len(ds):4d}  {statistics.fmean(ds):8.3f}  {statistics.pstdev(ds):9.3f}")
    preds = [float(r["pred_err_m"]) for r in rows_all if r.get("pred_err_m") not in (None, "", "nan")]
    if preds:
        print(f"비전 정지 예측 오차: 평균 {statistics.fmean(preds) * 100:.1f} cm · 최대 {max(preds) * 100:.1f} cm ({len(preds)}회)")
    f = fit([(float(r["speed_mps"]), float(r["dist_m"])) for r in rows_all])
    if not f:
        print("식 맞추기 불가 — 서로 다른 속도 2개 이상, 거리가 속도에 따라 늘어나야 함")
        return
    a, d0, rmse = f
    print(f"\n맞춘 식: d = v²/(2·{a:.3f}) {d0:+.3f}   (감속도 a = {a:.3f} m/s², 오차 RMSE {rmse * 100:.1f} cm)")
    print("목표 거리 → 추천 밀기 속도")
    for d in (0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50):
        v = math.sqrt(max(2 * a * (d - d0), 0.0))
        print(f"  {d:4.2f} m → {v:.3f} m/s")
    print(f"→ skills.yaml mode1 에 레인별 decel_mps2: {a:.3f}, d0_m: {d0:.3f} 로 기록")


class Runner:
    def __init__(self, lane, manual):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rally_interfaces.action import PushBall
        from rally_interfaces.msg import TrackedObjectArray
        self.rclpy, self.PushBall = rclpy, PushBall
        rclpy.init()
        self.node = Node("rally_push_recipe")
        self.ac = ActionClient(self.node, PushBall, "/rally/skill/push_ball")
        self.lock = threading.Lock()
        self.ball = None          # 최신 공 (TrackedObject)
        self.first_pred = None    # 이번 시도에서 처음 나온 정지 예측
        if not manual:
            self.node.create_subscription(TrackedObjectArray, "/rally/perception/objects", self._on_objects, qos_profile_sensor_data)
        self.ex = MultiThreadedExecutor(2)
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        self.lane = lane

    def _on_objects(self, msg):
        balls = [o for o in msg.objects if o.class_name == "ball"]
        if not balls:
            return
        b = max(balls, key=lambda o: o.confidence)
        with self.lock:
            self.ball = (time.monotonic(), b)
            if b.is_moving and self.first_pred is None and b.prediction_confidence > 0.3:
                self.first_pred = (b.predicted_stop.x, b.predicted_stop.y)

    def push(self, speed):
        if not self.ac.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/rally/skill/push_ball 없음 — skill_server 확인")
        with self.lock:
            self.first_pred = None
        goal = self.PushBall.Goal(lane=self.lane, push_speed=float(speed))
        fut = self.ac.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.05)
        gh = fut.result()
        if not gh.accepted:
            raise RuntimeError("goal 거부 — 안전 STOP 인지 확인 (scripts/estop.sh reset)")
        rf = gh.get_result_async()
        while not rf.done():
            time.sleep(0.05)
        res = rf.result().result
        if not res.success:
            raise RuntimeError(f"밀기 실패: {res.failure_reason}")

    def wait_stop(self, timeout_s=8.0, still_s=0.5):
        """공이 움직이다 멈춘 뒤 still_s 동안 is_moving=false 면 그 위치."""
        t0 = time.monotonic()
        still_since = None
        while time.monotonic() - t0 < timeout_s:
            time.sleep(0.05)
            with self.lock:
                b = self.ball
            if b is None or time.monotonic() - b[0] > 0.5:
                still_since = None
                continue
            if b[1].is_moving:
                still_since = None
            elif still_since is None:
                still_since = time.monotonic()
            elif time.monotonic() - still_since >= still_s:
                return b[1].position.x, b[1].position.y
        return None

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lane", type=int, default=1)
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.08, 0.12, 0.16, 0.20])
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--manual", action="store_true", help="비전 없이 줄자로 잰 정지 거리(cm) 입력")
    ap.add_argument("--fit", help="기존 CSV 만 다시 맞추기")
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])

    if opt.fit:
        with open(opt.fit, encoding="utf-8") as f:
            report(list(csv.DictReader(f)))
        return 0

    sx, sy, dir_deg, name = load_lane(opt.lane)
    ux, uy = math.cos(math.radians(dir_deg)), math.sin(math.radians(dir_deg))
    os.makedirs(ROOT / "data/measure", exist_ok=True)
    path = ROOT / f"data/measure/push_recipe_lane{opt.lane}_{datetime.now():%m%d_%H%M%S}.csv"
    plan = [v for v in opt.speeds for _ in range(opt.repeats)]
    print(f"레인 {opt.lane}({name}) 입구 ({sx:.3f}, {sy:.3f}) m · 방향 {dir_deg:.0f}° · 시도 {len(plan)}회 · "
          f"정지 위치 {'줄자 입력' if opt.manual else '비전 자동'} → {path}")
    r = Runner(opt.lane, opt.manual)
    rows = []
    try:
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            w.writeheader()
            for i, v in enumerate(plan, 1):
                input(f"\n[{i}/{len(plan)}] 속도 {v:.2f} m/s — 공을 레인 입구에 놓고 Enter ")
                r.push(v)
                pred = None
                if opt.manual:
                    s = input("  레인 입구에서 공 중심까지 정지 거리 [cm] (건너뛰려면 빈칸): ").strip()
                    if not s:
                        continue
                    dist, lat = float(s) / 100.0, float("nan")
                    stop = (sx + ux * dist, sy + uy * dist)
                    src = "manual"
                else:
                    stop = r.wait_stop()
                    if stop is None:
                        print("  정지 위치 못 받음 (비전 확인) — 이 시도 건너뜀")
                        continue
                    dx, dy = stop[0] - sx, stop[1] - sy
                    dist, lat = dx * ux + dy * uy, -dx * uy + dy * ux
                    pred = r.first_pred
                    src = "vision"
                perr = math.dist(pred, stop) if pred else float("nan")
                row = dict(t=f"{time.time():.3f}", lane=opt.lane, speed_mps=v, stop_x=f"{stop[0]:.4f}", stop_y=f"{stop[1]:.4f}",
                           dist_m=f"{dist:.4f}", lateral_m=f"{lat:.4f}", pred_x=f"{pred[0]:.4f}" if pred else "",
                           pred_y=f"{pred[1]:.4f}" if pred else "", pred_err_m=f"{perr:.4f}" if pred else "", source=src)
                w.writerow(row)
                f.flush()
                rows.append(row)
                print(f"  정지 거리 {dist * 100:.1f} cm" + ("" if math.isnan(lat) else f" · 좌우 {lat * 100:+.1f} cm")
                      + (f" · 예측 오차 {perr * 100:.1f} cm" if pred else ""))
    except KeyboardInterrupt:
        print("\n중단")
    except RuntimeError as e:
        print(f"\n중단: {e}")
    finally:
        r.close()
    print(f"\n저장 {path} ({len(rows)}회)")
    if rows:
        report(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
