#!/usr/bin/env python3
"""시뮬 전용 그리퍼 + 공 — /rally/gripper/command 를 받아 가상 RG2(/onrobot/sendCommand, bringup mode:=virtual)로 넘기고,
실로봇 gripper_node 처럼 /rally/gripper/state 를 발행하며, RViz 에 공·레인·테이블(/rally/sim/scene)을 그린다.
실제 그리퍼(Modbus 192.168.1.1)에는 붙지 않는다. 시뮬에서는 rally_robot gripper_node 대신 이것을 띄운다 (scripts/sim_robot.sh 가 띄움).

    python3 scripts/sim_gripper.py
    ros2 service call /rally/sim/give_ball std_srvs/srv/Trigger      # 손에 공 쥐여 주기 (실로봇에서 손으로 공을 대고 닫는 것과 같음)
    ros2 service call /rally/sim/place_ball std_srvs/srv/Trigger     # 공을 레인 2 입구에 놓기 (빈손으로 밀기 시험)

공 흉내: 쥐면 손가락 끝(TCP)을 따라다니고, 놓으면 그 자리에 떨어진다. 손가락이 공 뒤에서 밀면 따라 밀리고,
손가락이 멈추면 마지막 속도로 굴러가다 감속도 decel_mps2(기본 0.08 m/s²)로 멈춘다. 레인 벽·충돌은 흉내 내지 않는다.
"""
import argparse
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from onrobot_rg_msgs.srv import SetCommand
from rally_interfaces.srv import GripperCommand

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "ros2_ws/src/rally_bringup/config"
TCP_OFFSET = (0.0008, -0.0014, 0.2259)        # link_6 프레임 기준 TCP (print_pose.py 와 같음)


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # v' = q v q*
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


class SimGripper(Node):
    def __init__(self, ball_mm: float, decel: float):
        super().__init__("rally_sim_gripper")
        skills = yaml.safe_load((CFG / "skills.yaml").read_text(encoding="utf-8"))
        table = yaml.safe_load((CFG / "table.yaml").read_text(encoding="utf-8"))["table"]
        self.lanes = skills["mode1"]["lanes"]
        self.table_z = float(table.get("z_m") or 0.0)
        self.r = ball_mm / 2000.0
        self.decel = decel
        self.width = 110.0              # 현재 손가락 폭 mm
        self.held = False
        self.ball = None                # [x, y, z] base_link — 없으면 화면에 안 그림
        self.ball_v = (0.0, 0.0)        # 굴러가는 속도 (m/s)
        self.tip_hist = []              # [(monotonic, x, y)] 최근 0.25 s 손가락 끝 위치
        self.lock = threading.Lock()
        cbg = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.cli = self.create_client(SetCommand, "/onrobot/sendCommand", callback_group=cbg)
        self.create_service(GripperCommand, "/rally/gripper/command", self.on_cmd, callback_group=cbg)
        self.create_service(Trigger, "/rally/sim/give_ball", self.on_give, callback_group=cbg)
        self.create_service(Trigger, "/rally/sim/place_ball", self.on_place, callback_group=cbg)
        self.pub_state = self.create_publisher(JointState, "/rally/gripper/state", qos_profile_sensor_data)
        self.pub_scene = self.create_publisher(MarkerArray, "/rally/sim/scene", 5)
        self.create_timer(0.02, self.step, callback_group=cbg)
        self.create_timer(0.05, self.publish_state, callback_group=cbg)
        self.get_logger().info(f"sim gripper ready — 공 지름 {ball_mm:.0f} mm, 감속도 {decel} m/s² · /rally/sim/give_ball · /rally/sim/place_ball")

    # ----- 그리퍼 명령 -----
    def on_cmd(self, req, res):
        target = min(max(float(req.width_mm), 0.0), 110.0)
        with self.lock:
            ball_mm = self.r * 2000.0
            if self.held and target >= ball_mm:          # 열기 → 놓기
                self.held = False
                self.ball[2] = self.table_z + self.r
                self.ball_v = (0.0, 0.0)
            grab = (not self.held) and target < ball_mm and self._ball_between_fingers()
            if grab:
                self.held = True
            actual = ball_mm if self.held else target
            self.width = actual
            holding = self.held
        if self.cli.wait_for_service(timeout_sec=0.5):
            ev = threading.Event()
            fut = self.cli.call_async(SetCommand.Request(command=str(int(actual * 10))))
            fut.add_done_callback(lambda _f: ev.set())
            ev.wait(5.0 if req.wait_until_done else 0.0)
        res.success, res.actual_width_mm, res.object_detected = True, float(actual), bool(holding)
        self.get_logger().info(f"width {target:.0f} → {actual:.0f} mm, 공 쥠 {holding}")
        return res

    def on_give(self, req, res):
        tip = self._tip()
        with self.lock:
            if tip is None:
                res.success, res.message = False, "TCP 위치(tf) 없음"
                return res
            self.held, self.width = True, self.r * 2000.0
            self.ball = list(tip[1:])
        self.cli.call_async(SetCommand.Request(command=str(int(self.width * 10))))
        res.success, res.message = True, "공을 손에 쥐여 줌"
        return res

    def on_place(self, req, res):
        ln = self.lanes[1]
        with self.lock:
            self.held = False
            self.ball = [float(ln["start"][0]), float(ln["start"][1]), self.table_z + self.r]
            self.ball_v = (0.0, 0.0)
        res.success, res.message = True, f"공을 {ln.get('name', '레인 2')} 입구에 놓음"
        return res

    # ----- 물리 흉내 -----
    def _tip(self):
        try:
            tr = self.tf.lookup_transform("base_link", "link_6", Time(), timeout=Duration(seconds=0.05))
        except Exception:                                   # noqa: BLE001
            return None
        q = tr.transform.rotation
        o = tr.transform.translation
        dx, dy, dz = quat_rotate((q.x, q.y, q.z, q.w), TCP_OFFSET)
        stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9     # 속도는 TF 시각으로 (타이머 시각은 갱신 몰림에 튄다)
        return (stamp, o.x + dx, o.y + dy, o.z + dz)

    def _ball_between_fingers(self):
        tip = self._tip()
        if tip is None or self.ball is None:
            return False
        return math.dist(tip[1:], self.ball) < self.r * 1.2

    def step(self):
        tip = self._tip()
        with self.lock:
            if tip is not None and self.held and self.ball is not None:
                self.ball = list(tip[1:])
            elif self.ball is not None and tip is not None and not self.held:
                bx, by, bz = self.ball
                # 손가락 속도 = 최근 0.25 s 창의 이동 거리 / 시간 (TF 갱신 간격이 튀어도 안정)
                now = time.monotonic()
                self.tip_hist.append((now, tip[1], tip[2]))
                while self.tip_hist and now - self.tip_hist[0][0] > 0.25:
                    self.tip_hist.pop(0)
                vx = vy = 0.0
                if len(self.tip_hist) >= 3 and self.tip_hist[-1][0] - self.tip_hist[0][0] > 0.1:
                    t0, x0, y0 = self.tip_hist[0]
                    dt = self.tip_hist[-1][0] - t0
                    vx, vy = (tip[1] - x0) / dt, (tip[2] - y0) / dt
                touching = tip[3] < self.table_z + 2 * self.r and math.hypot(tip[1] - bx, tip[2] - by) < self.r + 0.012
                speed = math.hypot(vx, vy)
                if touching and speed > 0.005:
                    ux, uy = vx / speed, vy / speed
                    # 손가락 앞면에 붙어 밀림: 공 중심 = 손가락 + 진행 방향으로 (반지름 + 손가락 두께 반)
                    self.ball = [tip[1] + ux * (self.r + 0.012), tip[2] + uy * (self.r + 0.012), self.table_z + self.r]
                    self.ball_v = (vx, vy)
                else:
                    v = math.hypot(*self.ball_v)
                    if v > 1e-4:
                        dt = 0.02
                        nv = max(v - self.decel * dt, 0.0)
                        k = nv / v
                        self.ball_v = (self.ball_v[0] * k, self.ball_v[1] * k)
                        self.ball[0] += self.ball_v[0] * dt
                        self.ball[1] += self.ball_v[1] * dt
            ball = list(self.ball) if self.ball else None
        self.publish_scene(ball)

    # ----- 발행 -----
    def publish_state(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ["rg2_width_mm", "rg2_grip"]
        with self.lock:
            m.position = [float(self.width), 1.0 if self.held else 0.0]
        self.pub_state.publish(m)

    def publish_scene(self, ball):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        def mk(mid, typ, pos, scale, rgba, ns="scene"):
            m = Marker()
            m.header.frame_id, m.header.stamp = "base_link", now
            m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
            m.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            return m

        arr.markers.append(mk(0, Marker.CUBE, (0.42, -0.04, self.table_z - 0.01), (1.25, 1.02, 0.02), (0.92, 0.84, 0.67, 1.0)))
        mid = 1
        for i, ln in enumerate(self.lanes):
            x0, yc = float(ln["start"][0]) - 0.03, float(ln["start"][1])
            col = (0.29, 0.47, 0.82, 1.0) if i == 1 else (0.25, 0.75, 0.5, 1.0)
            for side in (+1, -1):
                arr.markers.append(mk(mid, Marker.CUBE, (x0 + 0.28, yc + side * 0.0585, self.table_z + 0.004), (0.56, 0.030, 0.008), col))
                mid += 1
        if ball is not None:
            arr.markers.append(mk(100, Marker.SPHERE, tuple(ball), (2 * self.r,) * 3, (0.95, 0.48, 0.10, 1.0), "ball"))
        self.pub_scene.publish(arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ball-mm", type=float, default=63.0)
    ap.add_argument("--decel", type=float, default=0.08, help="공 감속도 [m/s²] (레시피 실험 전 추정값)")
    ap.add_argument("--object-mm", type=float, default=None, help="(이전 옵션 호환) --ball-mm 과 같음")
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    rclpy.init()
    n = SimGripper(opt.object_mm or opt.ball_mm, opt.decel)
    ex = MultiThreadedExecutor(4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
