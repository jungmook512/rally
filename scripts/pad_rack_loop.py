#!/usr/bin/env python3
"""모드 3 패드 집기 → 꽂기 반복 — 스킬 서버 서비스(/rally/robot/pad_pick · pad_place)를 번갈아 부른다.
실제 스킬 코드(rally_robot/pad_rack.py: 순응 들기·대기 자세 경유·순응 꽂기)를 그대로 타므로 받기 없이 꽂기 동작만 반복 확인할 때 쓴다.

    source scripts/robot.sh env
    python3 scripts/pad_rack_loop.py                    # 5회, 회차 사이 Enter
    python3 scripts/pad_rack_loop.py --count 10 --auto  # 10회 연속 (회차 사이 --pause 초)
    python3 scripts/pad_rack_loop.py --start place      # 패드를 이미 쥐고 있으면 꽂기부터

전제: 터미널 2 (scripts/robot.sh nodes, 비전 없어도 됨) — skill_server·safety_monitor·force_publisher. 안전 NORMAL.
      시작 때 패드는 거치대에 (--start place 면 그리퍼에). skills.yaml 을 바꿨으면 터미널 2 재시작 (skill_server 가 시작 때 읽음).
한 회: pad_pick (A → G → 쥐기 → 순응으로 U → 대기 자세) → pad_place (대기 자세 → U → P → 순응 하강 → 놓기 → 빠짐).
출력: 단계별 시간·성공·사유, 스킬 서버의 [pad_rack] 로그(하강 끝 위치·힘 변화 최대·G 위 몇 mm), 단계 중 툴 힘 최대(/rally/robot/wrench).
실패·안전 STOP 이면 반복을 멈춘다 (해제는 scripts/robot.sh reset — 이 스크립트는 해제하지 않는다).
"""
import argparse
import math
import re
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import Trigger

from rally_interfaces.msg import SafetyStatus

LEVEL = {SafetyStatus.LEVEL_NORMAL: "NORMAL", SafetyStatus.LEVEL_SLOW: "SLOW", SafetyStatus.LEVEL_STOP: "STOP"}


class PadLoop(Node):
    def __init__(self):
        super().__init__("rally_pad_rack_loop")
        self.cli = {w: self.create_client(Trigger, f"/rally/robot/pad_{w}") for w in ("pick", "place")}
        self.safety = None
        self.f_peak = 0.0
        self.notes: list[str] = []                       # 이번 단계의 [pad_rack] 로그
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
        if msg.name.endswith("rally_skill_server") and "[pad_rack]" in msg.msg:
            print(f"    | {msg.msg}")
            self.notes.append(msg.msg)

    def stopped(self) -> bool:
        return self.safety is not None and self.safety.level == SafetyStatus.LEVEL_STOP

    def run(self, what: str, timeout_s: float) -> dict:
        """서비스 한 번. 끝날 때까지 콜백(안전·힘·로그)을 돌린다."""
        self.f_peak, self.notes = 0.0, []
        t0 = time.monotonic()
        fut = self.cli[what].call_async(Trigger.Request())
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 > timeout_s:
                # 스킬 서버 쪽 동작은 그대로 진행될 수 있다 — 로봇을 보고 필요하면 scripts/robot.sh estop
                return dict(ok=False, msg=f"응답 없음 {timeout_s:.0f} s", dt=time.monotonic() - t0, f=self.f_peak, notes=self.notes)
        for _ in range(10):                              # 끝난 직후 로그가 조금 늦게 온다
            rclpy.spin_once(self, timeout_sec=0.02)
        r = fut.result()
        return dict(ok=bool(r and r.success), msg=r.message if r else "응답 없음", dt=time.monotonic() - t0, f=self.f_peak, notes=self.notes)


def seat_info(notes: list[str]) -> str:
    """꽂기 로그에서 'G 위 몇 mm' · '힘 변화 최대' 를 뽑아 한 줄로."""
    force, seat, blocked = "", "", []                    # 힘은 마지막 하강만, 걸림은 높이 목록으로
    for s in notes:
        m = re.search(r"힘 변화 최대 ([\d.]+) N.*?(?:영점 합력 ([\d.]+) N)?(?: · 합력 ([\d.]+) N 으로 멈춤)?$", s)
        if m:
            force = f"힘 변화 {m.group(1)} N" + (f" (영점 {m.group(2)} N)" if m.group(2) else "") + (f" · 합력 {m.group(3)} N 멈춤" if m.group(3) else "")
        m = re.search(r"G 위 (\d+) mm 에서 걸림", s)
        if m:
            blocked.append(m.group(1))
        m = re.search(r"꽂힘 \((\w+)\).*G 위 (-?[\d.]+) mm\)(?: · (옆 .*?) \((\d+)번째\))?", s)
        if m:
            seat = f"{m.group(1)} · G 위 {m.group(2)} mm" + (f" · {m.group(3)} · {m.group(4)}번째" if m.group(3) else "")
    out = [x for x in (seat, f"걸림 {len(blocked)}번 (G 위 {', '.join(blocked)} mm)" if blocked else "", force) if x]
    return " · ".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=5, help="반복 횟수 (집기+꽂기 = 1회)")
    ap.add_argument("--auto", action="store_true", help="회차 사이 Enter 없이 --pause 초 쉬고 다음")
    ap.add_argument("--pause", type=float, default=2.0)
    ap.add_argument("--start", choices=("pick", "place"), default="pick", help="place = 이미 패드를 쥐고 있음")
    ap.add_argument("--timeout", type=float, default=120.0, help="한 단계 응답 기다림 [s]")
    opt = ap.parse_args()

    rclpy.init()
    n = PadLoop()
    rows = []
    try:
        for w, c in n.cli.items():
            if not c.wait_for_service(timeout_sec=15.0):   # Discovery Server 모드면 몇 초 걸림
                raise SystemExit(f"/rally/robot/pad_{w} 없음 — 터미널 2 (scripts/robot.sh nodes) 확인")
        t0 = time.monotonic()
        while n.safety is None and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        if n.safety is None:
            raise SystemExit("/rally/safety/status 못 받음")
        if n.safety.level != SafetyStatus.LEVEL_NORMAL:
            raise SystemExit(f"안전 {LEVEL.get(n.safety.level)} ({n.safety.reason}) — NORMAL 일 때만. scripts/robot.sh reset")

        steps = ["pick", "place"] if opt.start == "pick" else ["place", "pick"]
        name = {"pick": "집기", "place": "꽂기"}
        for i in range(1, opt.count + 1):
            if i > 1 or not opt.auto:
                if opt.auto:
                    time.sleep(opt.pause)
                elif input(f"\n[{i}/{opt.count}] Enter 진행, q 멈춤: ").strip().lower() == "q":
                    break
            row = {"i": i}
            for w in steps:
                if n.stopped():
                    raise SystemExit(f"안전 STOP ({n.safety.reason}) — 멈춤")
                print(f"[{i}/{opt.count}] {name[w]} …")
                r = n.run(w, opt.timeout)
                row[w] = r
                print(f"  → {name[w]} {'성공' if r['ok'] else '실패'} ({r['msg']}) · {r['dt']:.1f} s · 툴 힘 최대 {r['f']:.1f} N"
                      + (f" · {seat_info(r['notes'])}" if w == "place" and seat_info(r["notes"]) else ""))
                if not r["ok"]:
                    rows.append(row)
                    where = "패드를 쥔 채 입구 위에 있을 수 있음" if w == "place" else "패드 상태(거치대·그리퍼) 눈으로 확인"
                    raise SystemExit(f"{name[w]} 실패: {r['msg']} — 반복 멈춤 ({where})")
            rows.append(row)
    except KeyboardInterrupt:
        print("\nCtrl+C — 멈춤 (지금 단계는 스킬 서버에서 계속될 수 있음)")
    finally:
        if rows:
            print("\n회차 | 집기 s · 힘 N | 꽂기 s · 힘 N | 꽂기 결과")
            for row in rows:
                cells = []
                for w in ("pick", "place"):
                    r = row.get(w)
                    cells.append(f"{r['dt']:5.1f} · {r['f']:4.1f}{'' if r['ok'] else ' ✗'}" if r else "   -")
                pl = row.get("place")
                print(f"{row['i']:4d} | {cells[0]} | {cells[1]} | {seat_info(pl['notes']) if pl else ''}"
                      + (f" {pl['msg']}" if pl and not pl["ok"] else ""))
            ok = sum(1 for row in rows if row.get("pick", {}).get("ok") and row.get("place", {}).get("ok"))
            print(f"성공 {ok}/{len(rows)}")
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
