#!/usr/bin/env python3
"""모드 3 대기 자세(skills.yaml mode3.ready_posj)로 안전하게 보내기 — 패드 거치대 근처에서 시작해도 작업 공간 밖으로 휘지 않게.

    source scripts/robot.sh env
    python3 scripts/mode3_go_ready.py            # 20 °/s
    python3 scripts/mode3_go_ready.py --vel 15

경로 (pad_rack.pick_pad 끝과 같은 순서, 10/13): 거치대 손잡이 위(G~U 축 근처)면 먼저 U 로 곧게 들어 올림 → 팔(J1~3)만 대기 자세 값으로
→ 손목(J4~6) 대기 자세로. 거치대 근처에서 대기 자세로 바로 관절 이동하면 TCP 가 y −0.64 m 까지 나가 workspace_limit STOP (10/13 계산).
그 밖의 자세에서는 순기구학으로 바로 가는 경로를 검사해 작업 공간 안이면 바로, 아니면 멈추고 알린다.
안전: pad_rack_try.py 와 같다 — /rally/safety/status 가 NORMAL 일 때만 출발, 이동 중 STOP 이면 move_stop. STOP 해제는 하지 않는다.
"""
import argparse
import os
import sys

import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ejt_wrench import Chain  # noqa: E402
from jog import Jog, workspace  # noqa: E402
from pad_rack_try import fkin, movej_abs, skills_cfg  # noqa: E402
from print_pose import TCP_OFFSET_MM  # noqa: E402


def tcp_m(chain, q_deg):
    _, T = chain.fk_frames(np.radians(q_deg))
    return T[:3, 3] + T[:3, :3] @ (TCP_OFFSET_MM / 1000.0)


def path_ok(chain, a, b, ws, n=41):
    """관절 직선 보간 a → b 동안 TCP 가 작업 공간 안인지. (ok, 가장 나쁜 점)"""
    for s in np.linspace(0.0, 1.0, n):
        p = tcp_m(chain, np.asarray(a, float) + (np.asarray(b, float) - np.asarray(a, float)) * s)
        if not all(ws[k][0] <= v <= ws[k][1] for k, v in zip("xyz", p)):
            return False, p
    return True, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vel", type=float, default=20.0, help="관절 속도 °/s")
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    rclpy.init()
    n = Jog()
    m3 = skills_cfg()["mode3"]
    ready, U, G = (list(map(float, m3[k])) for k in ("ready_posj", "pad_rack_lift_posj", "pad_rack_grasp_posj"))
    chain, ws = Chain(), workspace()
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    q = {}
    from ejt_wrench import JOINTS

    def on_js(m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            q.setdefault("q", [d[j] for j in JOINTS])
    n.create_subscription(JointState, "/dsr01/joint_states", on_js, qos_profile_sensor_data)
    n.wait(lambda: "q" in q and n.safety is not None and n.tcp is not None, 5.0)
    if "q" not in q or n.safety is None:
        raise SystemExit("joint_states 또는 /rally/safety/status 를 못 받음 — bringup·터미널 2 확인")
    cur = list(np.degrees(q["q"]))
    p_cur, p_g, p_u = tcp_m(chain, cur), tcp_m(chain, G), tcp_m(chain, U)
    axis = (p_u - p_g) / np.linalg.norm(p_u - p_g)
    off = p_cur - p_g
    lateral = np.linalg.norm(off - (off @ axis) * axis)
    print(f"현재 관절 {np.round(cur, 1).tolist()} · TCP {np.round(p_cur, 3).tolist()} m")
    steps = []
    if lateral < 0.05 and -0.05 <= off @ axis <= np.linalg.norm(p_u - p_g) + 0.02:
        print(f"거치대 손잡이 축 위 (축에서 {lateral * 100:.1f} cm, G 위 {off @ axis * 100:.0f} cm) → U 로 들어 올린 뒤 팔 먼저 대기 자세로")
        steps = [("U (들어 올림)", U), ("경유 (팔만 대기 자세)", ready[:3] + U[3:]), ("대기 자세", ready)]
    else:
        steps = [("대기 자세", ready)]
    def legs_ok(seq):
        prev = cur
        for name, tgt in seq:
            ok, bad = path_ok(chain, prev, tgt, ws)
            if not ok:
                return name, bad
            prev = tgt
        return None, None

    bad_name, bad_p = legs_ok(steps)
    if bad_name is not None and len(steps) == 1:
        # 10/13: 모드 1 자세(J1 0·팔 앞으로)에서 대기 자세로 바로 가면 팔꿈치가 뒤집히며 TCP 가 x 0.67 까지 나감 → 팔을 위로 세운 뒤
        # (J2·J3 0) 세운 채 J1·손목을 대기값으로 돌리고 내려온다. 세 구간 모두 순기구학으로 작업 공간 안 확인 (z 최대 1.26 m — 위쪽 여유 확인)
        # J3 를 -15° 로 살짝 굽힘: 완전히 편 자세(J3 0)는 특이점이라 힘 값이 끊겨 5 s 넘으면 wrench_lost STOP (10/13 실측)
        up = [cur[0], 0.0, -15.0, cur[3], cur[4], cur[5]]
        turn = [ready[0], 0.0, -15.0, ready[3], ready[4], ready[5]]
        alt = [("팔 위로 세움", up), ("세운 채 J1·손목 대기값", turn), ("대기 자세", ready)]
        if legs_ok(alt)[0] is None:
            print(f"바로 가는 경로가 작업 공간 밖 (TCP {np.round(bad_p, 3).tolist()}) → 팔을 위로 세워서 돌아가는 경로 (z 최대 약 1.3 m)")
            steps, bad_name = alt, None
    if bad_name is not None:
        raise SystemExit(f"{bad_name} 로 가는 경로가 작업 공간 밖 (TCP {np.round(bad_p, 3).tolist()}) — 안 움직임. 직접교시로 옮겨 주세요")
    for name, tgt in steps:
        print(f"▶ {name}")
        movej_abs(n, tgt, opt.vel, fkin(n, tgt))
    print("대기 자세 도착")
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
