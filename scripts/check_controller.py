#!/usr/bin/env python3
"""두산 컨트롤러(DRCF) 건강 점검 — bringup 직후, force_publisher 띄우기 전에 10 s 안에 PASS/FAIL 로 판정.

    source scripts/env_robot_pc.sh && python3 scripts/check_controller.py
    python3 scripts/check_controller.py --apply-tool      # set_tool/set_tcp 재적용까지 시도 (로봇은 움직이지 않음)

판정 항목 (2026-10-06~07 증상 기준):
  1. robot_mode / robot_state 응답           — 안 오면 드라이버·연결 문제
  2. get_current_solution_space 3 s 내 응답   — DRCF 명령 채널이 멈추면 이것만 영원히 안 옴 (joint_states 는 정상)
  3. 툴·TCP 이름                              — 'Tool Weight_2FG' / '2FG_TCP' 가 아니면 펜던트에서 선택
  4. get_tool_force 3회 모두 0 이 아님        — 전부 0 이면 컨트롤러가 툴 힘(ETT)을 0 으로 보내는 상태 (2026-10-07 미해결)
  5. get_external_torque 0 이 아님            — 모니터링 스트림 자체 확인
  6. force_publisher 프로세스 0~1개            — 2개면 서비스 과부하 (project-dsr-service-overload)
종료 코드 0 = 전부 PASS (force_publisher·스킬 실행 가능), 1 = FAIL 있음.
"""
import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import (GetCurrentSolutionSpace, GetCurrentTcp, GetCurrentTool, GetExternalTorque,
                           GetRobotMode, GetRobotState, GetToolForce, SetCurrentTcp, SetCurrentTool, SetRobotMode)

PREFIX = "/dsr01/dsr_controller2/"
TOOL, TCP = "Tool Weight_2FG", "2FG_TCP"
MODE = {0: "MANUAL", 1: "AUTONOMOUS", 2: "MEASURE"}
STATE = {0: "INITIAL", 1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF", 4: "TEACHING", 5: "SAFE_STOP", 6: "EMERGENCY_STOP",
         7: "HOMMING", 8: "RECOVERY", 9: "SAFE_STOP2", 10: "SAFE_OFF2", 15: "NOT_READY"}


class Checker(Node):
    def __init__(self):
        super().__init__("rally_check_controller")
        self.fails = 0

    def call(self, srv_type, name, req, timeout_s=3.0):
        """서비스 1회 호출. 서버 없음 → None, 시간 초과 → 'TIMEOUT', 정상 → 응답."""
        cli = self.create_client(srv_type, PREFIX + name)
        if not cli.wait_for_service(timeout_sec=5.0):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        return fut.result() if fut.done() else "TIMEOUT"

    def item(self, ok, label, detail=""):
        print(("  [PASS] " if ok else "  [FAIL] ") + label + (f"  — {detail}" if detail else ""))
        if not ok:
            self.fails += 1
        return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply-tool", action="store_true", help="set_tool/set_tcp 재적용 시도")
    opt = ap.parse_args()
    rclpy.init()
    n = Checker()
    try:
        t0 = time.monotonic()
        r = n.call(GetRobotMode, "system/get_robot_mode", GetRobotMode.Request())
        dt = time.monotonic() - t0
        if r is None:
            n.item(False, "dsr_controller2 서비스 없음", "bringup 이 떠 있는지, env_robot_pc.sh 를 source 했는지")
            return 1
        n.item(r != "TIMEOUT" and r.success, f"robot_mode ({dt:.1f} s)", MODE.get(getattr(r, "robot_mode", -1), str(r)))
        r = n.call(GetRobotState, "system/get_robot_state", GetRobotState.Request())
        st = getattr(r, "robot_state", -1)
        n.item(r != "TIMEOUT" and st in (1, 2), "robot_state", STATE.get(st, str(r)) + ("" if st in (1, 2) else " → 펜던트에서 서보 온/복구"))

        t0 = time.monotonic()
        r = n.call(GetCurrentSolutionSpace, "aux_control/get_current_solution_space", GetCurrentSolutionSpace.Request(), 3.0)
        n.item(r not in (None, "TIMEOUT"), f"명령 채널 (get_current_solution_space {time.monotonic()-t0:.1f} s)",
               "" if r not in (None, "TIMEOUT") else "DRCF 명령 채널 무응답 → bringup 재시작, 그래도면 컨트롤러 전원 재부팅")

        tool = n.call(GetCurrentTool, "tool/get_current_tool", GetCurrentTool.Request())
        tcp = n.call(GetCurrentTcp, "tcp/get_current_tcp", GetCurrentTcp.Request())
        tn, cn = getattr(tool, "info", "?"), getattr(tcp, "info", "?")
        n.item(tn == TOOL and cn == TCP, "툴·TCP", f"tool='{tn}' tcp='{cn}'" + ("" if (tn, cn) == (TOOL, TCP) else f" → 펜던트에서 {TOOL}/{TCP} 선택"))
        if opt.apply_tool:
            # DRCF 는 set_tool/set_tcp 를 수동(MANUAL) 모드에서만 받는다 → 잠깐 바꿨다가 자동으로 복귀
            n.call(SetRobotMode, "system/set_robot_mode", SetRobotMode.Request(robot_mode=0))
            a = n.call(SetCurrentTool, "tool/set_current_tool", SetCurrentTool.Request(name=TOOL))
            b = n.call(SetCurrentTcp, "tcp/set_current_tcp", SetCurrentTcp.Request(name=TCP))
            m = n.call(SetRobotMode, "system/set_robot_mode", SetRobotMode.Request(robot_mode=1))
            n.item(getattr(a, "success", False) and getattr(b, "success", False) and getattr(m, "success", False),
                   "set_tool/set_tcp 재적용 (수동→자동)", f"{getattr(a, 'success', a)} / {getattr(b, 'success', b)} / 자동 복귀 {getattr(m, 'success', m)}")

        forces = []
        for _ in range(3):
            r = n.call(GetToolForce, "aux_control/get_tool_force", GetToolForce.Request(ref=0))
            forces.append(list(r.tool_force) if r not in (None, "TIMEOUT") else None)
            time.sleep(0.2)
        nonzero = any(f is not None and any(abs(v) > 1e-6 for v in f) for f in forces)
        last = forces[-1]
        n.item(nonzero, "get_tool_force ≠ 0", (f"|F|=({last[0]:.2f},{last[1]:.2f},{last[2]:.2f}) N" if last else str(forces))
               + ("" if nonzero else " → 컨트롤러가 툴 힘을 0 으로 보냄 (관절 외력은 정상이면 컨트롤러 설정 문제)"))
        r = n.call(GetExternalTorque, "aux_control/get_external_torque", GetExternalTorque.Request())
        ejt = list(r.ext_torque) if r not in (None, "TIMEOUT") else []
        n.item(any(abs(v) > 1e-6 for v in ejt), "get_external_torque ≠ 0", f"{[round(v, 2) for v in ejt]}" if ejt else str(r))

        ps = subprocess.run(["pgrep", "-fc", "rally_robot/force_publisher$"], capture_output=True, text=True)
        cnt = int(ps.stdout.strip() or 0)
        n.item(cnt <= 1, "force_publisher 프로세스 수", f"{cnt}개" + ("" if cnt <= 1 else " → 하나만 남기고 kill"))
    finally:
        n.destroy_node()
        rclpy.shutdown()
    print("요약: " + ("전부 PASS — force_publisher·스킬 실행 가능" if n.fails == 0 else f"{n.fails}개 FAIL — 위 화살표(→) 조치 후 재실행"))
    return 1 if n.fails else 0


if __name__ == "__main__":
    sys.exit(main())
