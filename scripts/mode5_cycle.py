#!/usr/bin/env python3
"""모드 5 반복 시험 — brain 없이 로봇 동작만: 집기 → 무작위 각도로 건네기 → (환자가 잡고 당김) → 돌려받기 → 집었던 자리에 내려놓기 → 다음 물체.

    source scripts/env_robot_pc.sh   (skill_server · safety_monitor · force_publisher · gripper_node 가 떠 있을 것)
    python3 scripts/mode5_cycle.py --cycles 4                                   # 비전(/rally/perception/objects)의 rod·cup 사용
    python3 scripts/mode5_cycle.py --cycles 2 --manual rod:0.30,0.20,0 cup:0.30,-0.20,0   # 비전 없이 x,y[m],yaw[deg] 직접
    python3 scripts/mode5_cycle.py --cycles 2 --manual rod:0.30,0.20,0 --tilt 20 40 --seed 1

각도: 매 회 |tilt| 를 --tilt MIN MAX 범위에서 고르고 부호(좌/우)도 무작위. 로봇은 skills.yaml handover.tilt_range_deg 로 한 번 더 클램프.
점수는 brain 이 낸다 — 여기서는 제시 각도와 잡을 때 손목 각도(HandState, 없으면 0)의 차이만 출력한다.
"""
import argparse
import math
import random
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rally_interfaces.action import Handover, PickCompliant
from rally_interfaces.msg import TrackedObjectArray

GRIP_N = {"rod": 20.0, "cup": 20.0}


class Cycle(Node):
    def __init__(self, use_vision):
        super().__init__("rally_mode5_cycle")
        self.pick = ActionClient(self, PickCompliant, "/rally/skill/pick_compliant")
        self.hand = ActionClient(self, Handover, "/rally/skill/handover")
        self.objects = {}
        self.lock = threading.Lock()
        if use_vision:
            self.create_subscription(TrackedObjectArray, "/rally/perception/objects", self._on_obj, qos_profile_sensor_data)

    def _on_obj(self, msg):
        with self.lock:
            for o in msg.objects:
                if o.class_name in ("rod", "cup") and not o.is_moving and o.confidence > 0.5:
                    self.objects[o.class_name] = (o.position.x, o.position.y, math.degrees(o.yaw), time.monotonic())

    def run(self, client, goal, label, feedback=True):
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"{label} 액션 서버 없음 — skill_server 확인")

        def fb(m):
            f = m.feedback
            if hasattr(f, "tool_force_n"):
                print(f"\r   [{label}] {f.phase:8s} 힘 {f.tool_force_n:5.1f} N   ", end="", flush=True)
            else:
                print(f"\r   [{label}] {f.phase:8s}            ", end="", flush=True)

        fut = client.send_goal_async(goal, feedback_callback=fb if feedback else None)
        while not fut.done():
            time.sleep(0.05)
        gh = fut.result()
        if not gh.accepted:
            raise RuntimeError(f"{label} 거부 — 안전 STOP 인지 확인 (scripts/estop.sh reset)")
        rf = gh.get_result_async()
        while not rf.done():
            time.sleep(0.05)
        print()
        return rf.result().status, rf.result().result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=4)
    ap.add_argument("--manual", nargs="*", help="class:x,y,yaw_deg (예 rod:0.30,0.20,0) — 비전 대신")
    ap.add_argument("--tilt", type=float, nargs=2, default=(15.0, 40.0), metavar=("MIN", "MAX"))
    ap.add_argument("--pull-n", type=float, default=0.0, help="당김 임계 [N] (0 = skills.yaml 기본)")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=None)
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    rng = random.Random(opt.seed)

    rclpy.init()
    n = Cycle(use_vision=not opt.manual)
    ex = MultiThreadedExecutor(2)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    manual = {}
    for item in opt.manual or []:
        cls, xyz = item.split(":")
        x, y, yaw = (float(v) for v in xyz.split(","))
        manual[cls] = (x, y, yaw)
    order = list(manual) if manual else ["rod", "cup"]
    results = []
    try:
        for i in range(opt.cycles):
            cls = order[i % len(order)]
            if manual:
                x, y, yaw = manual[cls]
            else:
                t0 = time.monotonic()
                while time.monotonic() - t0 < 5.0:
                    with n.lock:
                        o = n.objects.get(cls)
                    if o and time.monotonic() - o[3] < 1.0:
                        break
                    time.sleep(0.1)
                else:
                    print(f"[{i + 1}] {cls} 를 비전에서 못 찾음 — 건너뜀")
                    continue
                x, y, yaw = o[:3]
            tilt = rng.uniform(*opt.tilt) * rng.choice((-1, 1))
            print(f"\n[{i + 1}/{opt.cycles}] {cls} @ ({x:.3f}, {y:.3f}) yaw {yaw:.0f}° → 제시 각도 {tilt:+.0f}°")

            g = PickCompliant.Goal()
            g.target = PoseStamped()
            g.target.header.frame_id = "base_link"
            g.target.pose.position.x, g.target.pose.position.y = x, y
            g.yaw = math.radians(yaw)
            g.grip_force_n = GRIP_N.get(cls, 20.0)
            g.z_stiffness = 300.0
            st, r = n.run(n.pick, g, "집기")
            if not r.success:
                print(f"   집기 실패 (폭 {r.grip_width_mm:.0f} mm) — 중단")
                break

            g = Handover.Goal(direction=Handover.Goal.DIR_GIVE, object_class=cls, tilt_deg=float(tilt),
                              release_force_threshold_n=opt.pull_n, timeout_s=opt.timeout)
            print("   → 환자: 기울어진 물체를 손목을 돌려 잡고 당기세요")
            st, give = n.run(n.hand, g, "건네기")
            if not give.success:
                print(f"   건네기 실패: {give.failure_reason} — 중단")
                break

            print("   → 환자: 물체를 그리퍼 사이로 다시 밀어 넣으세요")
            g = Handover.Goal(direction=Handover.Goal.DIR_RECEIVE, object_class=cls, tilt_deg=0.0,
                              release_force_threshold_n=opt.pull_n, timeout_s=opt.timeout)
            st, recv = n.run(n.hand, g, "받기")
            diff = abs(give.wrist_angle_at_grasp_deg - tilt) if give.wrist_angle_at_grasp_deg else float("nan")
            results.append((cls, tilt, give.wrist_angle_at_grasp_deg, diff, give.handover_time_s, recv.success))
            print(f"   제시 {tilt:+.0f}° · 손목 {give.wrist_angle_at_grasp_deg:+.0f}° · 차이 {diff:.0f}° · 건네기 {give.handover_time_s:.1f} s · "
                  f"받기·내려놓기 {'성공' if recv.success else '실패 ' + recv.failure_reason}")
            if not recv.success:
                break
    except KeyboardInterrupt:
        print("\n중단 (로봇 동작은 skill_server 에서 계속될 수 있음 — 필요하면 scripts/estop.sh)")
    except RuntimeError as e:
        print(f"\n중단: {e}")
    finally:
        ex.shutdown()
        n.destroy_node()
        rclpy.try_shutdown()
    if results:
        print("\n요약")
        for cls, tilt, wrist, diff, t, ok in results:
            print(f"  {cls:4s} 제시 {tilt:+4.0f}° 손목 {wrist:+4.0f}° 차이 {diff:4.0f}° 건네기 {t:4.1f} s 받기 {'O' if ok else 'X'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
