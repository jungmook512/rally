#!/usr/bin/env python3
"""두 카메라 시각 차(dt_side) 스윕 — 같은 투구 bag 을 dt 값만 바꿔 flight_predictor 에 다시 돌려 어느 값이 맞는지 본다. 로봇 없이 (도메인 77).

    source scripts/robot.sh env
    python3 scripts/mode3_dt_sweep.py --bag data/bags/mode3_dt_1013_000000
    python3 scripts/mode3_dt_sweep.py --bag ... --dt -10 0 5 10 15 20 25 30 --reproj 6

원리: 측면 프레임 시각 + dt = 손목 시계. dt 가 틀리면 손목 픽셀을 엉뚱한 순간으로 보간해 두 시선이 안 만난다 →
      재투영 탈락이 늘고 3D 점·예측이 줄며 예측이 흔들린다. 맞는 dt 에서 탈락이 가장 적고 결정 시점(0.25 s) std 가 작고 예측 변화가 작다.
값마다: 예측 노드(호스트 venv, numpy 1)를 -p dt_side_s 로 띄움 → bag 재생(입력 토픽만, 녹화 때의 예측 토픽은 제외) → 노드 로그 통계 + landing_info 수집.
출력: dt 별 [투구 수, 공 검출 프레임, 3D 점, 재투영 탈락, 발행, 좋은 예측(std ≤ 10 cm) 첫 시각 중앙값, 0.25 s 시점 std 중앙값, 0.25 s → 마지막 예측점 변화 중앙값].
결과 적용: models/mode3_side_calib.json 의 dt_side_to_wrist_s 를 고른 값으로 (또는 노드 -p dt_side_s).
"""
import argparse
import os
import re
import signal
import statistics
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENV_PY = os.path.expanduser("~/venvs/yolo_bench/bin/python")
NP1 = os.path.expanduser("~/.local/rally_np1")
PRED = os.path.join(ROOT, "ros2_ws/install/rally_vision/lib/rally_vision/flight_predictor")
CALIB = os.path.join(ROOT, "models/mode3_side_calib.json")
INPUT_TOPICS = ["/camera/color/image_raw", "/camera/color/camera_info", "/side/side/color/image_raw", "/side/side/color/camera_info",
                "/rally/robot/tcp_pose", "/tf", "/tf_static", "/dsr01/joint_states"]
STAT_RE = re.compile(r"공 검출 프레임 측면 (\d+) 손목 (\d+) · 3D 점 (\d+) · 발행 (\d+) · 패드 위 공 \d+ · 재투영 탈락 누적 (\d+)")


def env77():
    e = dict(os.environ)
    e.update(ROS_DOMAIN_ID="77", ROS_LOCALHOST_ONLY="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
             PYTHONPATH=NP1 + (":" + e["PYTHONPATH"] if e.get("PYTHONPATH") else ""))
    e.pop("ROS_DISCOVERY_SERVER", None)
    return e


class Collector:
    """landing_info 를 받아 투구별 [ (since_release, std_max, n_obs, point) ] 로 모음 (1 s 넘게 끊기면 다음 투구)."""

    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Float64MultiArray
        from geometry_msgs.msg import PointStamped
        self.rclpy = rclpy
        rclpy.init(args=["--ros-args", "-r", "__node:=rally_dt_sweep"])
        self.node = rclpy.create_node("rally_dt_sweep")
        self.throws, self.cur, self.last_t, self.last_pt = [], [], None, None
        self.node.create_subscription(Float64MultiArray, "/rally/perception/landing_info", self._on_info, qos_profile_sensor_data)
        self.node.create_subscription(PointStamped, "/rally/perception/landing", self._on_pt, qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()

    def _spin(self):
        while self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def _on_pt(self, m):
        self.last_pt = (m.point.x, m.point.y, m.point.z)

    def _on_info(self, m):
        d = list(m.data)
        if len(d) < 6:
            return
        now = time.monotonic()
        with self.lock:
            if self.last_t is None or now - self.last_t > 1.0:
                if self.cur:
                    self.throws.append(self.cur)
                self.cur = []
            self.last_t = now
            self.cur.append((d[5], max(d[2], d[3]), d[4], self.last_pt))

    def reset(self):
        with self.lock:
            self.throws, self.cur, self.last_t = [], [], None

    def result(self):
        with self.lock:
            th = list(self.throws) + ([self.cur] if self.cur else [])
        return th


def summarize(throws):
    first_good, std25, drift = [], [], []
    for t in throws:
        g = [s for s, std, n, _ in t if std <= 0.10 and n >= 4]
        if g:
            first_good.append(min(g))
        near = [x for x in t if x[0] <= 0.27]
        if near:
            s25 = near[-1]
            std25.append(s25[1])
            if s25[3] is not None and t[-1][3] is not None:
                a, b = s25[3], t[-1][3]
                drift.append(sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5)
    med = lambda v: statistics.median(v) if v else float("nan")
    return len(throws), med(first_good), med(std25), med(drift)


def run_one(bag, dt_ms, reproj, rate, col, log_dir):
    log_path = os.path.join(log_dir, f"dt_{dt_ms:+.0f}ms.log")
    log = open(log_path, "w")
    pred = subprocess.Popen([VENV_PY, PRED, "--ros-args", "-p", f"dt_side_s:={dt_ms / 1000.0}", "-p", f"side_calib_path:={CALIB}",
                             "-p", f"max_reproj_px:={float(reproj)}"], env=env77(), stdout=log, stderr=subprocess.STDOUT)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90:
        time.sleep(1.0)
        if "워밍업" in open(log_path, errors="ignore").read():
            break
        if pred.poll() is not None:
            raise SystemExit(f"예측 노드가 바로 죽음 — {log_path} 확인")
    else:
        pred.send_signal(signal.SIGINT)
        raise SystemExit(f"예측 노드 모델 로드 90 s 초과 — {log_path}")
    time.sleep(2.0)
    col.reset()
    play = subprocess.run(["ros2", "bag", "play", bag, "--rate", str(rate), "--topics", *INPUT_TOPICS], env=env77(),
                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(6.0)                                           # 마지막 5 s 통계 한 번 더
    pred.send_signal(signal.SIGINT)
    try:
        pred.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pred.kill()
    log.close()
    side = wrist = pts = pub = 0
    rej = 0
    for m in STAT_RE.finditer(open(log_path, errors="ignore").read()):
        side += int(m.group(1)); wrist += int(m.group(2)); pts += int(m.group(3)); pub += int(m.group(4)); rej = int(m.group(5))
    n, fg, s25, dr = summarize(col.result())
    return dict(dt=dt_ms, side=side, wrist=wrist, pts=pts, rej=rej, pub=pub, throws=n, first_good=fg, std25=s25, drift=dr, play_rc=play.returncode)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--dt", type=float, nargs="+", default=[-10, 0, 5, 10, 15, 20, 25, 30], help="ms (측면 → 손목 시계)")
    ap.add_argument("--reproj", type=float, default=6.0, help="재투영 허용 px (노드 기본 6 — 탈락 수가 dt 에 민감하게)")
    ap.add_argument("--rate", type=float, default=1.0)
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    bag = os.path.abspath(opt.bag)
    if not os.path.isdir(bag):
        raise SystemExit(f"bag 폴더 없음: {bag}")
    for p in (VENV_PY, PRED, CALIB):
        if not os.path.exists(p):
            raise SystemExit(f"없음: {p}")
    if not os.path.isdir(NP1):
        raise SystemExit(f"numpy 1 경로 없음: {NP1} — ~/venvs/yolo_bench/bin/python -m pip install --target {NP1} 'numpy<2'")
    os.environ.update(ROS_DOMAIN_ID="77", ROS_LOCALHOST_ONLY="1")
    os.environ.pop("ROS_DISCOVERY_SERVER", None)
    log_dir = os.path.join(ROOT, "data/measure", "dt_sweep_" + time.strftime("%m%d_%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)
    col = Collector()
    print(f"dt 스윕 — bag {os.path.basename(bag)} · dt {opt.dt} ms · 재투영 {opt.reproj:.0f} px · 로그 {log_dir}")
    print(f"{'dt ms':>6} {'투구':>4} {'검출 측면/손목':>14} {'3D 점':>6} {'탈락':>6} {'발행':>5} {'좋은예측 s':>10} {'std@0.25':>9} {'0.25→끝 변화':>12}")
    rows = []
    for dt in opt.dt:
        r = run_one(bag, dt, opt.reproj, opt.rate, col, log_dir)
        rows.append(r)
        print(f"{r['dt']:>+6.0f} {r['throws']:>4} {r['side']:>6}/{r['wrist']:<7} {r['pts']:>6} {r['rej']:>6} {r['pub']:>5} "
              f"{r['first_good']:>10.2f} {r['std25'] * 100:>8.1f}cm {r['drift'] * 100:>10.1f}cm", flush=True)
    good = [r for r in rows if r["pts"] > 0]
    if good:
        best = min(good, key=lambda r: (r["rej"], -r["pub"]))
        print(f"\n재투영 탈락이 가장 적은 dt = {best['dt']:+.0f} ms (탈락 {best['rej']}, 발행 {best['pub']}). "
              f"적용: models/mode3_side_calib.json dt_side_to_wrist_s: {best['dt'] / 1000.0:.3f}")
    with open(os.path.join(log_dir, "summary.csv"), "w") as f:
        f.write("dt_ms,throws,side,wrist,pts,rej,pub,first_good_s,std25_m,drift_m\n")
        for r in rows:
            f.write(f"{r['dt']},{r['throws']},{r['side']},{r['wrist']},{r['pts']},{r['rej']},{r['pub']},{r['first_good']},{r['std25']},{r['drift']}\n")
    col.rclpy.try_shutdown()


if __name__ == "__main__":
    main()
