#!/usr/bin/env python3
"""모드 3 로봇 응답 시험 (MODE3_BRIEF 세션 D) — 대기 자세에서 패드 면 안으로 amovel, 명령 → 움직임 시작·도착·지나침을 잰다.

방향 (base_link, 공구 자세 고정, 위치만):
  +y / -y : 좌우            +u / -u : 패드 면(45°, 투구자 쪽을 봄) 안 '비스듬히 위로·뒤로' / '아래로·앞으로'
측정은 /dsr01/joint_states(100 Hz) + 순기구학 TCP (tcp_speed.py 와 같은 Chain·TCP 오프셋), 수신 시각 기준.
  t_start  = 명령 → TCP 가 출발점에서 2 mm 벗어남      t_arrive = 명령 → 목표 2 mm 안에 들어와 머묾
  overshoot = 이동 방향으로 목표를 지나친 최대 거리      peak = 최고 TCP 속도
  force_peak_n = 이동~복귀 전 /rally/robot/wrench 힘 크기 최대, force_dpeak_n = 명령 직전 힘에서 바뀐 최대 (관성 가짜 힘 포함)

    source scripts/robot.sh env
    python3 scripts/mode3_reach_test.py                     # 계획만 (로봇 안 움직임): 목표점·관절 한계·필요 관절 속도·작업 공간
    python3 scripts/mode3_reach_test.py --run --dist 5 10 --acc 1000 --vel 500 --reps 3 --dirs +y -y
    # 후보 대기 자세로 먼저 movej 한 뒤 패드 평면 24방향 (--angles: 0° 오른쪽(−y) · 90° 위 · 180° 왼쪽 · 270° 아래, 최적화 스크립트와 같은 기준.
    #   평면은 그 자세의 실제 패드 법선(R·PAD_NORMAL_TOOL) 으로 잡는다. 10/14 대기 자세 최적화 시뮬 확인용)
    python3 scripts/mode3_reach_test.py --run --posj -58.7 7.0 100.2 98.8 63.6 -100.3 --angles 0 15 30 --dist 25 --vel 100 --acc 500
결과 CSV: data/measure/mode3_reach_MMDD_HHMM.csv (한 줄 = 한 번 이동).
안전: 대기 자세는 지금 로봇 자세. 매 이동 전 /rally/safety/status 가 NORMAL 이 아니면 멈춤. 매 이동 뒤 대기 자세로 천천히(--return-vel) 돌아감.
      safety.yaml 작업 공간(실행 때 읽음) 밖 목표는 계획에서 빼고 알림. 속도는 --vel × 안전 speed_override.
      이동 중(시험 이동·대기 자세 복귀 모두) STOP 이 오면 바로 정지 명령 → 정지 확인 후 끝냄 (10/12).
DSR_ROBOT2 는 rally_robot.dsr_helpers.init_dsr() 뒤에 import (CLAUDE.md).
"""
import argparse
import csv
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ejt_wrench import JOINTS, Chain  # noqa: E402
from print_pose import TCP_OFFSET_MM  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
JOINT_VMAX = np.array([150, 150, 180, 225, 225, 225.0])   # M0609 관절 속도 한계 deg/s
J3_MAX = 148.0
WORKSPACE_FALLBACK = dict(x=(-2.00, 0.57), y=(-0.60, 0.60), z=(0.0, 2.00))   # safety.yaml 을 못 읽을 때만 (10/11 값)


def load_workspace():
    """safety.yaml 작업 공간 (m) — 설치본 우선, 없으면 소스 트리. 상수로 두면 safety.yaml 이 바뀔 때 어긋난다 (#141, PM 정리 10/12)."""
    import yaml
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(Path(get_package_share_directory("rally_bringup")) / "config" / "safety.yaml")
    except Exception:  # noqa: BLE001
        pass
    cands.append(ROOT / "ros2_ws" / "src" / "rally_bringup" / "config" / "safety.yaml")
    for p in cands:
        if p.is_file():
            c = yaml.safe_load(p.read_text(encoding="utf-8"))["rally_safety_monitor"]["ros__parameters"]
            return {a: (float(c[f"workspace_{a}_min_m"]), float(c[f"workspace_{a}_max_m"])) for a in "xyz"}
    print("safety.yaml 을 못 찾음 — 작업 공간 기본값 사용")
    return dict(WORKSPACE_FALLBACK)


WORKSPACE = load_workspace()                 # 실행 중엔 안전 노드가 safety.yaml 로 따로 감시
PAD_TILT_DEG = 45.0
PAD_NORMAL_TOOL = np.array([-0.7071, 0.0, 0.7071])      # 공구 좌표 패드 법선 (pad_tcp_check.py · mode3_ready_pose_opt.py 와 같음)


def directions(angles_deg=None, R0=None):
    """이동 방향 (base_link 단위 벡터). 기본은 45° 고정 면의 ±y·±u, --angles 가 있으면 실제 패드 법선 평면의 각도별 방향."""
    if angles_deg is None:
        n = np.array([np.cos(np.radians(PAD_TILT_DEG)), 0.0, np.sin(np.radians(PAD_TILT_DEG))])   # 패드 면 법선 (투구자 쪽 위)
        y = np.array([0.0, 1.0, 0.0])
        u = np.cross(n, y)                                       # 면 안 '비스듬히 위·뒤'
        return {"+y": y, "-y": -y, "+u": u, "-u": -u}
    n = R0 @ PAD_NORMAL_TOOL
    up = np.array([0.0, 0.0, 1.0])
    u = up - n * (up @ n); u /= np.linalg.norm(u)              # 평면 안 '위'
    r = np.cross(n, u); r /= np.linalg.norm(r)                 # 평면 안 '오른쪽'(−y 쪽) — 최적화와 같은 기준
    return {f"{a:g}°": np.cos(np.radians(a)) * r + np.sin(np.radians(a)) * u for a in angles_deg}


class Kin:
    def __init__(self):
        self.c = Chain()
        self.off = TCP_OFFSET_MM / 1000.0

    def tcp(self, q_rad):
        _, T = self.c.fk_frames(q_rad)
        return T[:3, 3] + T[:3, :3] @ self.off, T[:3, :3]

    def ik(self, p, R, seed):
        def err(q):
            pp, RR = self.tcp(q)
            return np.r_[pp - p, 0.3 * Rotation.from_matrix(R @ RR.T).as_rotvec()]
        r = least_squares(err, seed, xtol=1e-12, ftol=1e-12)
        e = err(r.x)
        return r.x if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 3e-4 else None

    def joint_rate_for(self, q, v):
        """TCP 를 v (m/s, 자세 고정) 로 움직일 때 관절 속도 deg/s (수치 자코비안)."""
        eps = 1e-5
        p0, R0 = self.tcp(q)
        J = np.zeros((6, 6))
        for i in range(6):
            dq = q.copy(); dq[i] += eps
            p1, R1 = self.tcp(dq)
            J[:3, i] = (p1 - p0) / eps
            J[3:, i] = Rotation.from_matrix(R1 @ R0.T).as_rotvec() / eps
        return np.degrees(np.linalg.solve(J, np.r_[v, 0, 0, 0]))


def in_workspace(p):
    return all(WORKSPACE[a][0] <= v <= WORKSPACE[a][1] for a, v in zip("xyz", p))


def plan(kin, q0, dists_cm, dirs, vel_mm_s, angles_deg=None):
    p0, R0 = kin.tcp(q0)
    dirmap = directions(angles_deg, R0)
    out = []
    for name in dirmap if angles_deg is not None else dirs:
        d = dirmap[name]
        for dc in dists_cm:
            tgt = p0 + d * dc / 100.0
            q, worst, ok, why = q0.copy(), 0.0, True, ""
            for s in np.linspace(0.0, 1.0, max(2, int(dc / 0.5)) + 1)[1:]:   # 직선 경로 0.5 cm 마다 (특이점을 건너뛰지 않게)
                q = kin.ik(p0 + d * dc / 100.0 * s, R0, q)
                if q is None:
                    ok, why = False, "IK 실패(도달 불가)"; break
                if abs(np.degrees(q[2])) > J3_MAX:
                    ok, why = False, f"J3 {np.degrees(q[2]):.0f}° > {J3_MAX}"; break
                rate = np.abs(kin.joint_rate_for(q, d * vel_mm_s / 1000.0)) / JOINT_VMAX
                worst = max(worst, rate.max())
            if ok and not in_workspace(tgt):
                ok, why = False, "안전 작업 공간 밖"
            if ok and worst > 3.0:
                ok, why = False, f"경로가 특이점을 지남 (관절 속도 한계의 {worst * 100:.0f}%) — 이 방향·거리는 이 자세로 불가"
            elif ok and worst > 0.8:
                ok, why = False, f"관절 속도 한계의 {worst * 100:.0f}% (> 80%) — --vel 낮출 것"
            out.append(dict(dir=name, dist_cm=dc, target=tgt, d=d, q_end=None if q is None else np.degrees(q),
                            joint_rate_pct=worst * 100, ok=ok, why=why))
    return p0, R0, out


class Watch(Node):
    def __init__(self, kin):
        super().__init__("rally_mode3_reach_watch")
        self.kin, self.lock = kin, threading.Lock()
        self.samples = []          # (t, p)
        self.forces = []           # (t, F xyz N)
        self.q = None
        self.safety = None
        self.create_subscription(JointState, "/dsr01/joint_states", self.on_js, qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self.on_wrench, qos_profile_sensor_data)
        from rally_interfaces.msg import SafetyStatus
        self.create_subscription(SafetyStatus, "/rally/safety/status", lambda m: setattr(self, "safety", m),
                                 QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10))
        # 받기 창 흉내 (10/13): 받기 스킬은 투구 감지 ~ 교차 + 0.5 s 동안 contact_expected + catch_window 하트비트(5 Hz)를 켜
        # 힘 한계가 130 N(safety.yaml catch_window_force_limit_n)이 된다. 가속 2000 시험 이동은 관성 가짜 힘이 25 N 을 넘어
        # 그 창 없이는 force_limit STOP (10/13 실측) → 시험 이동 동안만 같은 하트비트를 낸다. 복귀 이동은 끈다.
        from std_msgs.msg import Bool
        reliable1 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)
        self._pub_contact = self.create_publisher(Bool, "/rally/safety/contact_expected", reliable1)
        self._pub_window = self.create_publisher(Bool, "/rally/safety/catch_window", reliable1)
        self._window_on = False
        threading.Thread(target=self._beat_loop, daemon=True).start()   # rclpy 타이머는 joint_states 100 Hz 콜백에 밀려 0.5 s 넘게 늦어
                                                                        # 창이 잠깐 닫혔다(10/13 실측: 25 N 복귀 직후 force_limit STOP) → 별도 스레드 10 Hz

    def _beat_loop(self):
        from std_msgs.msg import Bool
        while rclpy.ok():
            if self._window_on:
                self._pub_contact.publish(Bool(data=True))
                self._pub_window.publish(Bool(data=True))
            time.sleep(0.1)

    def catch_window(self, on: bool):
        from std_msgs.msg import Bool
        self._window_on = bool(on)
        self._pub_contact.publish(Bool(data=self._window_on))
        self._pub_window.publish(Bool(data=self._window_on))
        if on:
            time.sleep(0.25)                                   # safety_monitor 가 하트비트를 한 번 받은 뒤 출발

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if not all(k in d for k in JOINTS):
            return
        q = np.array([d[k] for k in JOINTS])
        p, _ = self.kin.tcp(q)
        with self.lock:
            self.q = q
            self.samples.append((time.monotonic(), p))

    def on_wrench(self, m):
        f = m.wrench.force
        with self.lock:
            self.forces.append((time.monotonic(), np.array([f.x, f.y, f.z])))
            del self.forces[:-3000]

    def force_stats(self, t_cmd, t_end):
        """t_cmd ~ t_end 사이 힘 크기 최대와 명령 직전 힘에서 바뀐 최대 (N). 힘을 못 받았으면 빈 dict."""
        with self.lock:
            before = [f for t, f in self.forces if t < t_cmd]
            during = [f for t, f in self.forces if t_cmd <= t <= t_end]
        if not during:
            return {}
        F = np.array(during)
        f0 = before[-1] if before else F[0]
        return dict(force_peak_n=float(np.linalg.norm(F, axis=1).max()),
                    force_dpeak_n=float(np.linalg.norm(F - f0, axis=1).max()), force_n_samples=len(during))

    def wait_normal(self, timeout_s=2.0):
        """로봇이 멈춰 있을 때 부른다. NORMAL 이 될 때까지 최대 timeout_s 기다려 안전 상태를 돌려줌 (아니면 마지막 상태).
        이동 중 force_publisher 가 두산 서비스 응답을 0.5 s 못 받으면 wrench_stale SLOW 가 잠깐 뜬다 (10/11 실측) — 그것 때문에 시험 전체가 끝나지 않게."""
        t = time.monotonic()
        while time.monotonic() - t < timeout_s:
            s = self.safety
            if s is not None and s.level == 0:
                return s
            time.sleep(0.05)
        return self.safety

    def take(self, since):
        with self.lock:
            return [(t, p) for t, p in self.samples if t >= since]

    def clear(self):
        with self.lock:
            self.samples = []


def analyse(samples, t_cmd, p0, tgt, d):
    if not samples:
        return {}
    t = np.array([s[0] for s in samples]) - t_cmd
    P = np.array([s[1] for s in samples])
    moved = np.linalg.norm(P - p0, axis=1)
    to_tgt = np.linalg.norm(P - tgt, axis=1)
    along = (P - p0) @ d
    dist = np.linalg.norm(tgt - p0)
    i0 = np.argmax(moved > 0.002) if (moved > 0.002).any() else None
    arrive = None
    inside = to_tgt < 0.002
    for i in range(len(inside)):
        if inside[i:].all():
            arrive = t[i]; break
    # 속도: 30 ms 넘게 떨어진 두 표본으로 (joint_states 가 받은 시각 기준으로 몰려 들어와 이웃 차분은 튄다 — 10/10 실측 4 m/s 오계산)
    v = []
    j = 0
    for i in range(len(t)):
        while j < len(t) and t[j] - t[i] < 0.03:
            j += 1
        if j >= len(t):
            break
        v.append(np.linalg.norm(P[j] - P[i]) / (t[j] - t[i]))
    v = np.array(v)
    return dict(t_start_ms=None if i0 is None else t[i0] * 1000, t_arrive_s=arrive,
                overshoot_mm=max(0.0, (along.max() - dist) * 1000), peak_mps=float(v.max()) if len(v) else 0.0,
                final_err_mm=float(to_tgt[-1] * 1000))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="실제로 로봇을 움직임 (없으면 계획만)")
    ap.add_argument("--dist", type=float, nargs="+", default=[5, 10, 20, 30], help="cm")
    ap.add_argument("--dirs", nargs="+", type=str.strip, default=["+y", "-y", "+u", "-u"], choices=["+y", "-y", "+u", "-u"])
    ap.add_argument("--acc", type=float, nargs="+", default=[1000.0], help="mm/s²")
    ap.add_argument("--vel", type=float, default=500.0, help="mm/s (안전 speed_override 를 곱함)")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--return-vel", type=float, default=100.0, help="대기 자세로 돌아가는 속도 mm/s")
    ap.add_argument("--settle", type=float, default=1.0, help="이동 사이 정지 시간 s")
    ap.add_argument("--posj", type=float, nargs=6, metavar="DEG", help="대기 자세 관절값 — 먼저 movej(20 °/s) 로 가서 거기서 시작 (없으면 지금 자세)")
    ap.add_argument("--angles", type=float, nargs="+", metavar="DEG", help="패드 평면 각도 (0 오른쪽 −y · 90 위 · 180 왼쪽 · 270 아래). 있으면 --dirs 대신")
    ap.add_argument("--posj-vel", type=float, default=20.0, help="--posj 이동 관절 속도 °/s")
    ap.add_argument("--ws-x-max", type=float, help="계획에서 쓸 작업 공간 x 상한 m (safety.yaml 대신). 안전 노드는 따로 param set 해야 함 — 시뮬·모드 3 시험용")
    # argparse 는 "-y"·"-u" 를 옵션 이름으로 읽는다 → 공백을 붙여 값으로 넘기고 type=str.strip 으로 되돌림
    opt = ap.parse_args([a + " " if a in ("-y", "-u") else a for a in sys.argv[1:]])

    rclpy.init()
    kin = Kin()
    w = Watch(kin)
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor()
    ex.add_node(w)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    t0 = time.monotonic()
    while w.q is None and time.monotonic() - t0 < 5:
        time.sleep(0.05)
    if w.q is None:
        print("/dsr01/joint_states 를 못 받음 — bringup 확인 (source scripts/robot.sh env)")
        return 1
    q0 = np.radians(opt.posj) if opt.posj else w.q.copy()
    if opt.ws_x_max is not None:
        WORKSPACE["x"] = (WORKSPACE["x"][0], opt.ws_x_max)
        print(f"작업 공간 x 상한 {opt.ws_x_max} m 로 계획 (안전 노드는 ros2 param set /rally_safety_monitor workspace_x_max_m {opt.ws_x_max} 필요)")
    p0, R0, steps = plan(kin, q0, opt.dist, opt.dirs, opt.vel, opt.angles)
    print(f"대기 자세 관절 {np.round(np.degrees(q0), 1)}  TCP {np.round(p0, 3)} m  패드 법선 {np.round(R0 @ PAD_NORMAL_TOOL, 3)} · vel {opt.vel:.0f} mm/s")
    print(f"{'방향':>4} {'cm':>4}  {'목표 TCP (m)':<24} {'관절속도':>8}  판정")
    for s in steps:
        print(f"{s['dir']:>4} {s['dist_cm']:>4.0f}  {str(np.round(s['target'], 3)):<24} {s['joint_rate_pct']:>7.0f}%  "
              f"{'OK' if s['ok'] else '빼기 — ' + s['why']}")
    todo = [s for s in steps if s["ok"]]
    if not opt.run:
        print(f"\n계획만 (로봇 안 움직임). 실행할 이동 {len(todo)}개 × 가속 {len(opt.acc)} × {opt.reps}회 = {len(todo) * len(opt.acc) * opt.reps}번")
        ex.shutdown()
        rclpy.try_shutdown()
        return 0

    from rally_robot.dsr_helpers import init_dsr
    dsr = init_dsr("rally_mode3_reach")
    from DSR_ROBOT2 import amovel, amovej, check_motion, get_current_posx, get_robot_state  # noqa: E402  init_dsr 뒤
    from rally_robot.dsr_helpers import stop_robot
    if opt.posj:
        # 후보 대기 자세로 관절 이동 (특이점 무관). 팔 모양이 크게 바뀌므로 느리게, 안전 STOP 폴링
        t_pj = time.monotonic()
        amovej(list(opt.posj), vel=opt.posj_vel, acc=opt.posj_vel * 2)
        while True:
            time.sleep(0.02)
            if w.safety is not None and w.safety.level == 2:
                stop_robot(dsr, w.get_logger(), f"safety STOP {w.safety.reason} (posj 이동)")
                raise SystemExit(f"대기 자세 이동 중 안전 STOP ({w.safety.reason})")
            if time.monotonic() - t_pj > 0.5 and check_motion() == 0:
                break
            if time.monotonic() - t_pj > 60:
                stop_robot(dsr, w.get_logger(), "posj 이동 시간 초과"); raise SystemExit("대기 자세 이동 60 s 초과")
        time.sleep(0.5)
        if get_robot_state() not in (1, 2):
            raise SystemExit(f"로봇 상태 {get_robot_state()} — posj 이동 뒤 알람")
        q_now = w.q.copy()
        err_deg = np.degrees(np.abs(q_now - q0)).max()
        if err_deg > 1.0:
            raise SystemExit(f"posj 도착 오차 {err_deg:.1f}° — 관절값 확인")
        print(f"대기 자세 도착: 관절 {np.round(np.degrees(q_now), 1)}  TCP {np.round(kin.tcp(q_now)[0], 3)} m")
    start6 = list(get_current_posx()[0])
    out = ROOT / "data" / "measure" / f"mode3_reach_{datetime.now():%m%d_%H%M%S}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []

    t1 = time.monotonic()                         # 안전 상태 구독이 들어올 때까지 (시작하자마자 첫 scale() 이 None 으로 끝나지 않게)
    while w.safety is None and time.monotonic() - t1 < 5.0:
        time.sleep(0.05)

    def guard(label):
        """이동 중 폴링: STOP 이면 바로 정지 명령 → 정지 확인 → 끝냄 (예전엔 이동 전에만 확인)."""
        s = w.safety
        if s is not None and s.level == 2:
            stopped = stop_robot(dsr, w.get_logger(), f"safety STOP {s.reason} ({label})")
            raise SystemExit(f"{label} 중 안전 STOP ({s.reason}) — 정지 {'확인' if stopped else '실패! 펜던트 확인'}")

    def wait_motion(t_cmd, label, limit_s):
        while True:
            time.sleep(0.01)
            guard(label)
            if time.monotonic() - t_cmd > 0.2 and check_motion() == 0:
                return
            if time.monotonic() - t_cmd > limit_s:
                stop_robot(dsr, w.get_logger(), f"{label} 시간 초과")
                raise SystemExit(f"{label}: {limit_s:.0f} s 안에 안 끝남 — 멈춤")

    def scale():
        s = w.wait_normal()
        if s is None:
            raise SystemExit("안전 상태(/rally/safety/status)를 못 받음 — 노드(robot.sh nodes) 확인")
        if s.level != 0:
            raise SystemExit(f"안전 상태가 NORMAL 아님 (level {s.level}, {s.reason}) — 멈춤")
        return float(s.speed_override)

    try:
        for acc in opt.acc:
            for s in todo:
                d = s["d"]
                tgt6 = list(start6)
                for i in range(3):
                    tgt6[i] = start6[i] + d[i] * s["dist_cm"] * 10.0      # mm
                for rep in range(opt.reps):
                    k = scale()
                    time.sleep(opt.settle)
                    w.clear()
                    t_cmd = time.monotonic()
                    t_wall = time.time()                       # bag 과 맞추는 실제 시각 (#195)
                    amovel(tgt6, vel=opt.vel * k, acc=acc * k)
                    wait_motion(t_cmd, "시험 이동", 10.0)
                    time.sleep(0.3)
                    r = analyse(w.take(t_cmd), t_cmd, p0, s["target"], d)
                    r.update(w.force_stats(t_cmd, time.monotonic()))
                    r["robot_state"] = get_robot_state()      # 1 STANDBY · 2 MOVING 아니면 두산 알람·정지 (10/11 −u 자기충돌)
                    row = dict(dir=s["dir"], dist_cm=s["dist_cm"], acc=acc, vel=opt.vel * k, rep=rep + 1, t_cmd_wall=f"{t_wall:.3f}", **r)
                    w.get_logger().info(f"amovel 명령 {t_wall:.3f} {s['dir']} {s['dist_cm']:.0f} cm acc {acc:.0f}")   # /rosout 에도 (#195)
                    rows.append(row)
                    print(f"{s['dir']} {s['dist_cm']:.0f} cm acc {acc:.0f} #{rep + 1}: 시작 {r.get('t_start_ms') or float('nan'):.0f} ms · "
                          f"도착 {r.get('t_arrive_s') or float('nan'):.3f} s · 지나침 {r.get('overshoot_mm', 0):.1f} mm · "
                          f"최고 {r.get('peak_mps', 0):.2f} m/s · 끝 오차 {r.get('final_err_mm', 0):.1f} mm · "
                          f"힘 최대 {r.get('force_peak_n', float('nan')):.1f} N (변화 {r.get('force_dpeak_n', float('nan')):.1f})")
                    if r["robot_state"] not in (1, 2):
                        raise SystemExit(f"로봇 상태 {r['robot_state']} (알람·정지) — 펜던트에서 해제 후 대기 자세로")
                    # 복귀도 amovel + 폴링 — 막는 movel·mwait 동안엔 두산 서비스가 막혀 wrench_stale SLOW 가 나고(정동휘 #144) STOP 도 못 봤다
                    kr = scale()
                    t_ret = time.monotonic()
                    amovel(start6, vel=opt.return_vel * kr, acc=opt.return_vel * 2 * kr)
                    dist_mm = s["dist_cm"] * 10.0
                    wait_motion(t_ret, "대기 자세 복귀", dist_mm / max(opt.return_vel * kr, 1.0) * 2.0 + 5.0)
    finally:
        if rows:
            with open(out, "w", newline="") as f:
                wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                wr.writeheader()
                wr.writerows(rows)
            print(f"\n저장: {out}")
        dsr.destroy_node()
        ex.shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
