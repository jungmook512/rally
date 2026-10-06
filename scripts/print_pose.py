#!/usr/bin/env python3
"""현재 로봇 자세를 posx(mm, ZYZ deg) · posj(deg) · m 단위로 출력 — 티칭 값을 skills.yaml / table.yaml 에 적을 때.

    source scripts/env_robot_pc.sh          # 두산 bringup 이 떠 있어야 함
    python3 scripts/print_pose.py           # 한 번 출력
    python3 scripts/print_pose.py --watch   # 2 Hz 로 계속 출력 (티치펜던트/직접교시로 움직이며 보기, Ctrl+C 로 종료)
"""
import argparse
import sys
import time

import rclpy
from rally_robot.dsr_helpers import init_dsr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", action="store_true")
    opt = ap.parse_args()
    rclpy.init()
    dsr = init_dsr("rally_print_pose")
    from DSR_ROBOT2 import get_current_posj, get_current_posx
    try:
        while True:
            px = [float(v) for v in get_current_posx()[0]]
            pj = [float(v) for v in get_current_posj()]
            print(f"posx (mm, deg): [{', '.join(f'{v:.1f}' for v in px)}]")
            print(f"posj (deg)    : [{', '.join(f'{v:.1f}' for v in pj)}]")
            print(f"xyz (m)       : [{px[0]/1000:.4f}, {px[1]/1000:.4f}, {px[2]/1000:.4f}]")
            if not opt.watch:
                break
            print("---")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        dsr.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
