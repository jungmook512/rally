#!/usr/bin/env python3
"""RG2 그리퍼 Modbus TCP 연결·읽기·쓰기 확인 (Day 0 todo 2번). 로봇·ROS 없이 PC1에서 바로 실행.

    python3 scripts/check_gripper.py                 # 연결 + 읽기 + 열고 닫기 왕복 + 지연 측정
    python3 scripts/check_gripper.py --no-move       # 연결 + 읽기만 (움직이지 않음)
    python3 scripts/check_gripper.py --measure       # 물체를 손가락 사이에 두고 실행 → 파지 폭 측정 (skills.yaml 기입용)
    python3 scripts/check_gripper.py --ip 192.168.1.1 --port 502

종료 코드 0 = 전부 통과. 실패하면 어느 단계에서 막혔는지와 점검 순서를 출력한다.
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

# colcon build 없이도 선생님 onrobot.RG 를 쓰기 위해 소스 경로를 직접 추가
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "rally_robot"))
from rally_robot.onrobot import RG  # noqa: E402

MOTION_BIT, GRIP_BIT = 0, 1


def step(ok: bool, msg: str) -> bool:
    print(("  [OK]   " if ok else "  [FAIL] ") + msg)
    return ok


def wait_motion_done(rg: RG, timeout_s: float = 3.0) -> float:
    """motion 비트가 꺼질 때까지 대기. 걸린 시간(s) 반환, 타임아웃이면 -1."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if not rg.get_status()[MOTION_BIT]:
            return time.monotonic() - t0
        time.sleep(0.02)
    return -1.0


def move_and_measure(rg: RG, width_mm: float, force_n: float) -> dict:
    """명령 → 폭이 1 mm 이상 변하기까지(반응 지연) · motion 끝까지(완료 시간) 측정."""
    w0 = rg.get_width()
    t0 = time.monotonic()
    rg.move_gripper(int(width_mm * 10), int(force_n * 10))
    react = -1.0
    while time.monotonic() - t0 < 3.0:
        if abs(rg.get_width() - w0) >= 1.0:
            react = time.monotonic() - t0
            break
        time.sleep(0.01)
    done = wait_motion_done(rg)
    w1 = rg.get_width()
    return dict(target=width_mm, actual=w1, react_s=react, done_s=done, grip=rg.get_status()[GRIP_BIT])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.1")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--no-move", action="store_true", help="읽기만 하고 움직이지 않음")
    ap.add_argument("--measure", action="store_true", help="물체 파지 폭 측정 (저힘으로 닫아 grip 감지 폭 기록)")
    ap.add_argument("--force", type=float, default=20.0, help="이동 힘 [N] (RG2: 3~40)")
    args = ap.parse_args()
    fails = 0

    print(f"1. TCP {args.ip}:{args.port}")
    try:
        with socket.create_connection((args.ip, args.port), timeout=2):
            pass
        step(True, "포트 열림 (컴퓨트 박스 응답)")
    except OSError as e:
        step(False, f"연결 안 됨: {e}")
        print("     점검: PC1 유선 IP 가 192.168.1.x 인지 → ping 192.168.1.1 → 브라우저 http://192.168.1.1 (웹 클라이언트) 에서 "
              "그리퍼 인식 여부 → 웹 클라이언트 Modbus TCP 활성화")
        return 1

    print("2. Modbus 연결 (pymodbus, slave 65)")
    rg = RG("rg2", args.ip, str(args.port))
    if not step(rg.connected, f"RG.connected = {rg.connected}"):
        return 1

    print("3. 레지스터 읽기")
    try:
        width = rg.get_width()
        status = rg.get_status()
        offset = rg.get_fingertip_offset()
        step(True, f"폭 {width:.1f} mm · 핑거팁 오프셋 {offset:.1f} mm · status {status}")
        if status[GRIP_BIT]:
            print("     (지금 뭔가 물고 있음 — grip detected)")
        if any(status[2:]):
            print("     주의: 안전 스위치/회로 비트가 켜져 있음 → 그리퍼가 안 움직일 수 있음. 손가락 안쪽 스위치 확인")
    except Exception as e:
        step(False, f"읽기 실패: {e}")
        print("     점검: 웹 클라이언트에서 Modbus TCP 가 켜져 있는지, 다른 프로그램(웹 클라이언트 제어 탭 등)이 점유 중인지")
        return 1

    if args.measure:
        print(f"4. 파지 폭 측정 — {args.force:.0f} N 으로 닫음. 물체가 손가락 사이에 있어야 함")
        r = move_and_measure(rg, 0.0, args.force)
        if step(r["grip"] == 1, f"grip detected = {r['grip']} · 닫힌 폭 {r['actual']:.1f} mm"):
            print(f"     → skills.yaml 파지 폭 후보: {r['actual'] - 2:.0f} mm (측정값 - 2) · 힘 {args.force:.0f} N")
        else:
            print("     grip 이 안 잡힘: 물체가 없거나 너무 작거나(최소 폭 0), 힘이 너무 약함. --force 30 으로 재시도")
            fails += 1
        rg.move_gripper(int(min(r["actual"] + 30, 110) * 10), int(args.force * 10))
        wait_motion_done(rg)
        return 1 if fails else 0

    if args.no_move:
        print("4. (--no-move) 이동 생략")
        return 0

    print(f"4. 쓰기 + 왕복 이동 ({args.force:.0f} N) — 손가락 사이 비워 두세요")
    results = []
    for target in (100.0, 30.0, 100.0):
        r = move_and_measure(rg, target, args.force)
        results.append(r)
        ok = r["done_s"] >= 0 and abs(r["actual"] - target) <= 3.0
        step(ok, f"목표 {target:5.1f} → 실제 {r['actual']:5.1f} mm · 반응 {r['react_s']*1000:5.0f} ms · 완료 {r['done_s']*1000:5.0f} ms")
        fails += 0 if ok else 1
    reacts = [r["react_s"] for r in results if r["react_s"] > 0]
    if reacts:
        print(f"     명령→폭 변화 지연 평균 {sum(reacts)/len(reacts)*1000:.0f} ms  ← todo '그리퍼 닫힘 지연' 항목, skills.yaml 에 기록")
    print("요약: " + ("전부 통과 — Day 0 2번 완료" if fails == 0 else f"{fails}개 실패"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
