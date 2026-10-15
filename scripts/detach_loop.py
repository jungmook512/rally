#!/usr/bin/env python3
"""모드 3 공 떼기만 반복 시험 — 스킬 서버 action(/rally/skill/detach_ball)을 부른다. 받기·꽂기 없이 떼기 동작만.
실제 스킬 코드(rally_robot/skills/detach_ball.py: 공 보기 자세 → 비전 공 → 세로로 쥠 → 벗기기 → 후퇴 → 박스)를 그대로 탄다.

    source scripts/env_robot_pc.sh
    python3 scripts/detach_loop.py                     # 1회
    python3 scripts/detach_loop.py --count 3           # 3회, 회차 사이 Enter (공을 패드에 다시 붙이고)
    python3 scripts/detach_loop.py --ball 0.30 -0.15 0.40   # 비전 대신 공 위치(base m) 직접
    python3 scripts/detach_loop.py --no-box            # 박스에 안 놓고 떼기까지만 (교시 없을 때 — skills.yaml detach_box_drop_posj 비우고)

전제: 터미널 1 bringup · 터미널 2 pc1_robot(skill_server·safety_monitor·force_publisher·gripper_node) · 비전 컨테이너(/rally/perception/objects).
      시작할 때 손목 인식(segmenter)을 모드 3(벨크로 공 모델 v2 만)으로 바꾸고 끝나면 되돌린다 (--ball 을 주면 안 바꿈).
      안전 NORMAL. 공을 붙인 패드가 거치대에 꽂혀 있고 그리퍼는 비어 있음. 로봇은 아무 자세나 (스킬이 공 보기 자세로 먼저 감).
      skills.yaml 을 바꿨으면 터미널 2 재시작 (skill_server 가 시작 때 읽음).
출력: 단계(phase)·힘 피드백, 스킬 서버의 [detach_ball]·[pad_rack] 로그, 툴 힘 최대(/rally/robot/wrench), 결과(성공·최대 당김 힘·급락).
실패·안전 STOP 이면 반복을 멈춘다 (해제: ros2 service call /rally/robot/reset_safety std_srvs/srv/Trigger — 이 스크립트는 해제하지 않는다).
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from rally_interfaces.action import DetachBall
from rally_interfaces.msg import SafetyStatus

LEVEL = {SafetyStatus.LEVEL_NORMAL: "NORMAL", SafetyStatus.LEVEL_SLOW: "SLOW", SafetyStatus.LEVEL_STOP: "STOP"}
STATUS = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


class DetachLoop(Node):
    def __init__(self):
        super().__init__("rally_detach_loop")
        self.cli = ActionClient(self, DetachBall, "/rally/skill/detach_ball")
        self.safety = None
        self.f_peak = 0.0
        self.notes: list[str] = []
        self.phases: list[tuple[float, str, float]] = []   # (t, phase, 힘)
        self.create_subscription(SafetyStatus, "/rally/safety/status", self._on_safety,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self._on_wrench, qos_profile_sensor_data)
        self.create_subscription(Log, "/rosout", self._on_log, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))

    def _on_safety(self, msg: SafetyStatus):
        if self.safety is not None and msg.level != self.safety.level:
            print(f"    안전 {LEVEL.get(self.safety.level, '?')} → {LEVEL.get(msg.level, '?')} ({msg.reason})")
        self.safety = msg

    def _on_wrench(self, msg: WrenchStamped):
        f = msg.wrench.force
        self.f_peak = max(self.f_peak, math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z))

    def _on_log(self, msg: Log):
        if msg.name.endswith("rally_skill_server") and ("[detach_ball]" in msg.msg or "[pad_rack]" in msg.msg or "[movej_safe]" in msg.msg):
            print(f"    | {msg.msg}")
            self.notes.append(msg.msg)

    def set_seg_mode(self, mode: int) -> int | None:
        """손목 인식(segmenter) active_mode 를 바꾸고 이전 값을 돌려준다. 실패하면 None (경고만). brain 없이 돌릴 때 모드 3 모델로."""
        from rcl_interfaces.msg import Parameter as P, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters
        get = self.create_client(GetParameters, "/rally_segmenter/get_parameters")
        put = self.create_client(SetParameters, "/rally_segmenter/set_parameters")
        if not (get.wait_for_service(timeout_sec=10.0) and put.wait_for_service(timeout_sec=5.0)):
            print("  ⚠ /rally_segmenter 파라미터 서비스 없음 — 손목 인식 모델을 못 바꿈 (vision 컨테이너 확인)")
            return None
        f = get.call_async(GetParameters.Request(names=["active_mode"]))
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        prev = f.result().values[0].integer_value if f.done() and f.result() and f.result().values else 0
        req = SetParameters.Request(parameters=[P(name="active_mode", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=mode))])
        f = put.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        r = f.result().results[0] if f.done() and f.result() and f.result().results else None
        if r is None or not r.successful:
            print(f"  ⚠ 손목 인식 모델 모드 {mode} 로 못 바꿈 ({getattr(r, 'reason', '응답 없음')})")
            return None
        print(f"  손목 인식 모델 → 모드 {mode} (이전 {prev})")
        return prev

    def stopped(self) -> bool:
        return self.safety is not None and self.safety.level == SafetyStatus.LEVEL_STOP

    def run(self, ball, grip_force: float, limit: float, timeout_s: float) -> dict:
        """action 한 번. 끝날 때까지 콜백(안전·힘·로그·피드백)을 돌린다."""
        self.f_peak, self.notes, self.phases = 0.0, [], []
        goal = DetachBall.Goal()
        if ball is not None:
            goal.ball_pose.header.frame_id = "base_link"
            goal.ball_pose.pose.position.x, goal.ball_pose.pose.position.y, goal.ball_pose.pose.position.z = map(float, ball)
        goal.grip_force_n, goal.pull_force_limit_n = float(grip_force), float(limit)
        t0 = time.monotonic()
        last = {"phase": None}

        def on_fb(fb_msg):
            fb = fb_msg.feedback
            if fb.phase != last["phase"]:
                last["phase"] = fb.phase
                self.phases.append((time.monotonic() - t0, fb.phase, float(fb.pull_force_n)))
                print(f"    단계 {fb.phase} ({time.monotonic() - t0:.1f} s, 힘 {fb.pull_force_n:.1f} N)")

        fut = self.cli.send_goal_async(goal, feedback_callback=on_fb)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 > 10.0:
                return dict(ok=False, msg="goal 응답 없음 10 s", dt=time.monotonic() - t0, f=self.f_peak, notes=self.notes)
        gh = fut.result()
        if not gh.accepted:
            return dict(ok=False, msg="goal 거부 (안전 STOP·다른 스킬 실행 중?)", dt=time.monotonic() - t0, f=self.f_peak, notes=self.notes)
        rfut = gh.get_result_async()
        while not rfut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 > timeout_s:
                # 스킬 서버 쪽 동작은 그대로 진행될 수 있다 — 로봇을 보고 필요하면 estop
                return dict(ok=False, msg=f"결과 없음 {timeout_s:.0f} s", dt=time.monotonic() - t0, f=self.f_peak, notes=self.notes)
        for _ in range(10):                                  # 끝난 직후 로그가 조금 늦게 온다
            rclpy.spin_once(self, timeout_sec=0.02)
        res = rfut.result()
        r = res.result
        reason = next((s.split("중단: ", 1)[1] for s in self.notes if "[detach_ball] 중단: " in s), "")
        return dict(ok=bool(r.success) and res.status == 4, msg=STATUS.get(res.status, str(res.status)) + (f" · {reason}" if reason else ""),
                    dt=time.monotonic() - t0, f=self.f_peak, notes=self.notes, peak=float(r.peak_pull_force_n), drop=float(r.detach_drop_n),
                    phases=list(self.phases))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=1, help="반복 횟수 (회차 사이 Enter — 공을 패드에 다시 붙이고)")
    ap.add_argument("--ball", type=float, nargs=3, metavar=("X", "Y", "Z"), help="공 위치 base [m] (비우면 비전 /rally/perception/objects)")
    ap.add_argument("--force", type=float, default=0.0, help="쥐는 힘 N (0 = skills.yaml gripper.velcro_ball)")
    ap.add_argument("--limit", type=float, default=0.0, help="당김 힘 한계 N (0 = skills.yaml mode3.detach_limit_n, 기본 15)")
    ap.add_argument("--timeout", type=float, default=180.0, help="한 회 결과 기다림 [s]")
    opt = ap.parse_args()

    rclpy.init()
    n = DetachLoop()
    rows = []
    prev_mode = None
    try:
        if not n.cli.wait_for_server(timeout_sec=15.0):      # Discovery Server 모드면 몇 초 걸림
            raise SystemExit("/rally/skill/detach_ball 없음 — 터미널 2 (pc1_robot) 확인")
        t0 = time.monotonic()
        while n.safety is None and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        if n.safety is None:
            raise SystemExit("/rally/safety/status 못 받음")
        if n.safety.level != SafetyStatus.LEVEL_NORMAL:
            raise SystemExit(f"안전 {LEVEL.get(n.safety.level)} ({n.safety.reason}) — NORMAL 일 때만 (reset_safety)")
        if opt.ball is None:
            prev_mode = n.set_seg_mode(3)                    # 모드 3 = 벨크로 공 모델(v2) 만 (10/15)
        for i in range(1, opt.count + 1):
            if i > 1 and input(f"\n[{i}/{opt.count}] 공을 패드에 붙였으면 Enter, q 멈춤: ").strip().lower() == "q":
                break
            if n.stopped():
                raise SystemExit(f"안전 STOP ({n.safety.reason}) — 멈춤")
            print(f"[{i}/{opt.count}] 공 떼기 … ({'비전' if opt.ball is None else '공 ' + str(opt.ball)})")
            r = n.run(opt.ball, opt.force, opt.limit, opt.timeout)
            rows.append(r)
            print(f"  → {'성공' if r['ok'] else '실패'} ({r['msg']}) · {r['dt']:.1f} s · 툴 힘 최대 {r['f']:.1f} N"
                  + (f" · 당김 최대 {r['peak']:.1f} N · 급락 {r['drop']:.1f} N" if "peak" in r else ""))
            if not r["ok"]:
                raise SystemExit("실패 — 반복 멈춤 (공·그리퍼 상태 눈으로 확인)")
    except KeyboardInterrupt:
        print("\nCtrl+C — 멈춤 (지금 동작은 스킬 서버에서 계속될 수 있음)")
    finally:
        if rows:
            print("\n회차 | 결과 | 시간 s | 툴 힘 최대 N | 당김 최대 N | 급락 N | 단계")
            for i, r in enumerate(rows, 1):
                ph = " → ".join(f"{p}@{t:.0f}s" for t, p, _ in r.get("phases", []))
                print(f"{i:4d} | {'성공' if r['ok'] else '실패 ' + r['msg']} | {r['dt']:5.1f} | {r['f']:4.1f} | "
                      f"{r.get('peak', float('nan')):4.1f} | {r.get('drop', float('nan')):4.1f} | {ph}")
            print(f"성공 {sum(1 for r in rows if r['ok'])}/{len(rows)}")
        if prev_mode is not None and prev_mode != 3:
            n.set_seg_mode(prev_mode)                        # 원래 모드로
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
