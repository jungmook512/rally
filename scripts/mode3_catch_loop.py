#!/usr/bin/env python3
"""모드 3 예측 받기 반복 — brain 없이 CatchBall 을 계속 보낸다. 던지기 → 예측 → 받으러 이동 → 복귀, 붙었든 아니든 다음 투구.

    source scripts/robot.sh env
    python3 scripts/mode3_catch_loop.py                  # Ctrl+C 까지 반복
    python3 scripts/mode3_catch_loop.py --count 10 --radius 0.5 --timeout 30

전제: 패드를 쥔 채(안 쥐었으면 스킬이 거치대에서 집음), 터미널 2 가 MODE3=1 로 떠 있음(두 카메라·flight_predictor), 안전 NORMAL.
      skills.yaml mode3.pad_place_after_catch: false (10/12 밤 시험 — 판정 뒤 거치대에 꽂지 않고 대기 자세로 복귀).
한 번: 대기 자세 → 공 던지기 기다림(timeout) → 공 놓은 뒤 decide_after_s 의 예측점으로 amovel 한 번 → 비전 판정(최대 judge_timeout_s) → 대기 자세.
출력: 예측 교차점, 패드 중심과의 차이, 로봇 이동 거리, 반응 시간, 결과(caught / out_of_radius / low_confidence / missed / timeout),
      TCP 최고 속도(/dsr01/joint_states 100 Hz 순기구학, 30 ms 떨어진 표본 차분 — mode3_reach_test 와 같은 방법)·툴 힘 최대(/rally/robot/wrench).
안전 STOP 이면 반복을 멈추고 사유를 보여준다 (해제는 scripts/robot.sh reset — 이 스크립트는 해제하지 않는다).
"""
import argparse
import math
import os
import select
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float64MultiArray

from rally_interfaces.action import CatchBall
from rally_interfaces.msg import SafetyStatus

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ejt_wrench import JOINTS, Chain  # noqa: E402  (mode3_reach_test 와 같은 순기구학 — /dsr01/joint_states 100 Hz 로 TCP 속도)
from print_pose import TCP_OFFSET_MM  # noqa: E402

LEVEL = {SafetyStatus.LEVEL_NORMAL: "NORMAL", SafetyStatus.LEVEL_SLOW: "SLOW", SafetyStatus.LEVEL_STOP: "STOP"}


def mode3_cfg():
    from ament_index_python.packages import get_package_share_directory
    return yaml.safe_load(open(f"{get_package_share_directory('rally_bringup')}/config/skills.yaml")).get("mode3") or {}


class Loop(Node):
    def __init__(self):
        super().__init__("rally_mode3_catch_loop")
        self.catch = ActionClient(self, CatchBall, "/rally/skill/catch_ball")
        self.safety = None
        self.n_pred = 0
        self.first_good = None                           # 이번 투구에서 std ≤ 10 cm·관측 ≥ 4 가 처음 나온 '놓은 뒤' 시각
        self.create_subscription(SafetyStatus, "/rally/safety/status", self._on_safety,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Float64MultiArray, "/rally/perception/landing_info", self._on_info, qos_profile_sensor_data)
        self.chain, self.off = Chain(), TCP_OFFSET_MM / 1000.0
        self.lock = threading.Lock()
        self.samples, self.forces = [], []                 # (t, TCP xyz m) · (t, |F| N) — goal 동안만 모음
        self.recording = False
        self.create_subscription(JointState, "/dsr01/joint_states", self._on_js, qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self._on_wrench, qos_profile_sensor_data)
        # 10/13: 공 떼기 안내를 스킬이 접촉 예정을 켠 순간(판정 뒤 대기 자세 5 s)에 띄움 — goal 이 끝난 뒤 띄우면 이미 창이 닫혀 nudge STOP
        self.judged, self.remove_told = False, False
        self.create_subscription(Bool, "/rally/safety/contact_expected", self._on_contact, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def _on_js(self, m):
        if not self.recording:
            return
        d = dict(zip(m.name, m.position))
        if not all(k in d for k in JOINTS):
            return
        _, T = self.chain.fk_frames(np.array([d[k] for k in JOINTS]))
        with self.lock:
            self.samples.append((time.monotonic(), T[:3, 3] + T[:3, :3] @ self.off))

    def _on_contact(self, m):
        if m.data and self.judged and not self.remove_told:
            self.remove_told = True
            print("   ▶ 지금 붙은 공 떼기 (5 s, 이 동안만 nudge 쉼)", flush=True)

    def _on_wrench(self, m):
        if self.recording:
            f = m.wrench.force
            with self.lock:
                self.forces.append((time.monotonic(), math.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2)))

    def motion_stats(self):
        """goal 동안의 TCP 최고 속도 [m/s]·툴 힘 최대 [N]. 속도는 30 ms 넘게 떨어진 두 표본 차분 (이웃 차분은 수신 몰림으로 튐)."""
        with self.lock:
            s, f = list(self.samples), list(self.forces)
        peak = 0.0
        j = 0
        for i in range(len(s)):
            while j < len(s) and s[j][0] - s[i][0] < 0.03:
                j += 1
            if j >= len(s):
                break
            peak = max(peak, float(np.linalg.norm(s[j][1] - s[i][1]) / (s[j][0] - s[i][0])))
        return peak, (max(v for _, v in f) if f else float("nan")), len(s)

    def _on_safety(self, m):
        self.safety = m

    def _on_info(self, m):
        d = list(m.data)
        if len(d) < 6:
            return
        self.n_pred += 1
        if self.first_good is None and max(d[2], d[3]) <= 0.10 and d[4] >= 4:
            self.first_good = d[5]

    def level(self):
        return None if self.safety is None else self.safety.level

    def wait_normal(self, timeout_s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if self.level() == SafetyStatus.LEVEL_NORMAL:
                return True
            time.sleep(0.1)
        return False

    def run_once(self, goal):
        phases = []
        self.judged, self.remove_told = False, False

        def fb(m):
            f = m.feedback
            if not phases or phases[-1] != f.phase:
                phases.append(f.phase)
                self.judged = f.phase in ("caught", "missed")
                extra = f" (예측 ({f.current_prediction.x:.2f}, {f.current_prediction.y:.2f}, {f.current_prediction.z:.2f}) m, 도착까지 {f.time_to_impact_s:.2f} s)" \
                    if f.phase in ("tracking", "intercept") else ""
                print(f"   [{f.phase}]{extra}", flush=True)

        with self.lock:
            self.samples, self.forces = [], []
        self.recording = True
        fut = self.catch.send_goal_async(goal, feedback_callback=fb)
        while not fut.done():
            time.sleep(0.05)
        gh = fut.result()
        if not gh.accepted:
            raise RuntimeError("goal 거부 — 안전 STOP 이거나 skill_server 가 다른 goal 실행 중")
        rf = gh.get_result_async()
        while not rf.done():
            time.sleep(0.05)
        self.recording = False
        return rf.result().status, rf.result().result, phases


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=0, help="반복 횟수 (0 = Ctrl+C 까지)")
    ap.add_argument("--radius", type=float, default=None, help="받으러 가는 반경 m (기본 skills.yaml mode3.catch_radius_m)")
    ap.add_argument("--timeout", type=float, default=30.0, help="투구 기다리는 시간 s")
    ap.add_argument("--no-ask", action="store_true", help="사람 판정(y + Enter) 묻지 않고 비전 판정만 (10/16 정동휘: 측면 판정이 잘 보임)")
    ap.add_argument("--after-catch-wait", type=float, default=0.0,
                    help="받은 뒤 다음 goal 전에 더 기다리는 시간 s. 10/13 부터 스킬이 받은 뒤 접촉 예정을 켜고 기다리므로(skills.yaml mode3.ball_removal_wait_s) 기본 0")
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    # 10/13 실로봇: 터미널 2 재시작 뒤 손목 카메라 노출이 자동으로 남아(컨테이너 안 고정이 카메라보다 먼저 돌아 실패한 것으로 봄)
    # 날아가는 공이 번져 손목 검출이 드묾 → 3D 점 5개에서 예측이 엉뚱 (피구 궤적). 시작할 때마다 두 카메라 노출을 다시 고정
    for cam in ("/camera", "/side/side"):
        r = subprocess.run(["ros2", "run", "rally_bringup", "set_camera_params", "--node", cam, "--fixed"], capture_output=True, text=True, timeout=60)
        print(f"노출 고정 {cam}: {'ok' if r.returncode == 0 and '노출 고정' in r.stdout + r.stderr else '실패 — ' + (r.stdout + r.stderr).strip()[-200:]}")
    cfg = mode3_cfg()
    radius = opt.radius if opt.radius is not None else float(cfg.get("catch_radius_m", 0.35))
    pad = [float(v) for v in (cfg.get("pad_center_base_m") or [math.nan] * 3)]
    if cfg.get("pad_place_after_catch", True):
        print("★ skills.yaml mode3.pad_place_after_catch 가 true — 받으면 거치대에 꽂으러 간다. 반복 시험이면 false 로 두고 터미널 2 재시작")

    rclpy.init()
    n = Loop()
    ex = MultiThreadedExecutor(2)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    if not n.catch.wait_for_server(timeout_sec=5.0):
        raise SystemExit("/rally/skill/catch_ball 액션 서버 없음 — 터미널 2 (MODE3=1 scripts/robot.sh nodes) 확인")
    if not n.wait_normal(3.0):
        raise SystemExit(f"안전 {LEVEL.get(n.level(), n.level())} ({getattr(n.safety, 'reason', '없음')}) — NORMAL 이어야 시작. scripts/robot.sh reset")
    print(f"예측 받기 반복 — 반경 {radius:.2f} m · 투구 대기 {opt.timeout:.0f} s · decide_after {cfg.get('decide_after_s', '?')} s"
          f" · 패드 중심(대기) {[round(v, 3) for v in pad]} m. Ctrl+C 로 끝")
    results = []
    i = 0
    try:
        while opt.count <= 0 or i < opt.count:
            i += 1
            if not n.wait_normal(1.0):
                print(f"\n안전 {LEVEL.get(n.level(), n.level())} ({n.safety.reason if n.safety else '?'}) — 반복 멈춤. scripts/robot.sh reset 뒤 다시")
                break
            print(f"\n=== 투구 {i} — 대기 자세로 가면 던지세요")
            n.n_pred, n.first_good = 0, None
            goal = CatchBall.Goal(catch_radius=float(radius), timeout_s=float(opt.timeout))
            t0 = time.monotonic()
            try:
                status, r, phases = n.run_once(goal)
            except RuntimeError as e:
                print(f"   {e}")
                break
            p = r.landing_predicted
            moved = r.robot_travel_mm > 1.0
            if r.failure_reason == "timeout":
                verdict = "투구 없음 (timeout)"
            elif r.caught:
                verdict = "★ 받음"
            else:
                verdict = {"out_of_radius": "반경 밖 — 안 움직임", "low_confidence": "예측 품질 미달 — 안 움직임",
                           "missed": "이동했지만 놓침" if moved else "놓침", "": "놓침" if not r.caught else ""}.get(r.failure_reason, r.failure_reason)
            d = "" if any(math.isnan(v) for v in pad) or r.failure_reason == "timeout" else \
                f" · 패드 중심 차이 x {(p.x - pad[0]) * 100:+.0f} y {(p.y - pad[1]) * 100:+.0f} z {(p.z - pad[2]) * 100:+.0f} cm"
            fg = "" if n.first_good is None else f" · 좋은 예측 놓은 뒤 {n.first_good:.2f} s"
            peak, fmax, ns = n.motion_stats()
            sp = f" · TCP 최고 {peak:.2f} m/s · 힘 최대 {fmax:.0f} N" if ns else " · (joint_states 없음 — 속도 못 잼)"
            print(f"   결과: {verdict} · 예측 {n.n_pred}개{fg}\n"
                  f"   예측 교차점 ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m{d}\n"
                  f"   로봇 이동 {r.robot_travel_mm:.0f} mm · 반응 {r.reaction_time_s:.2f} s{sp} · 단계 {' → '.join(phases)} · {time.monotonic() - t0:.0f} s")
            manual = None
            if moved and r.failure_reason != "timeout" and not opt.no_ask:
                # 10/13: 측면 판정이 패드 가운데 붙은 공을 못 보는 경우가 있어 사람 판정도 같이 기록. 스킬이 접촉 예정 켜고 기다리는 ball_removal_wait_s(5 s) 안에
                print("   ▶ 받았으면 y + Enter (4 s): ", end="", flush=True)
                rd, _, _ = select.select([sys.stdin], [], [], 4.0)
                ans = sys.stdin.readline().strip().lower() if rd else ""
                manual = ans.startswith("y")
                print(f"   사람 판정: {'받음' if manual else '놓침'} · 비전 판정: {'받음' if r.caught else '놓침'}")
            results.append((verdict, moved, r.caught, manual))
            if r.caught and opt.after_catch_wait > 0:
                print(f"   ★ 공을 패드에서 떼 주세요 — {opt.after_catch_wait:.0f} s 뒤 다음 투구 (떼는 힘이 nudge STOP 으로 잡히지 않게 goal 밖에서)", flush=True)
                time.sleep(opt.after_catch_wait)
            if n.level() == SafetyStatus.LEVEL_STOP:
                print(f"\n안전 STOP ({n.safety.reason}) — 반복 멈춤. 로봇 상태 확인 뒤 scripts/robot.sh reset")
                break
    except KeyboardInterrupt:
        print("\n끝 (Ctrl+C)")
    finally:
        nt = len(results)
        if nt:
            th = [x for x in results if not x[0].startswith("투구 없음")]
            print(f"\n===== 요약: goal {nt}회 · 투구 {len(th)}회 · 받으러 이동 {sum(1 for x in th if x[1])}회 · 비전 받음 {sum(1 for x in th if x[2])}회"
                  f" · 사람 판정 받음 {sum(1 for x in th if x[3])}회")
            for k, (v, _, c, m) in enumerate(results, 1):
                print(f"  {k}: {v}" + ("" if m is None else f" · 사람 판정 {'받음' if m else '놓침'}"))
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
