#!/usr/bin/env python3
"""시뮬 전용 그리퍼 + 공 — /rally/gripper/command 를 받아 가상 RG2(/onrobot/sendCommand, bringup mode:=virtual)로 넘기고,
실로봇 gripper_node 처럼 /rally/gripper/state 를 발행하며, RViz 에 공·레인·테이블(/rally/sim/scene)을 그린다.
실제 그리퍼(Modbus 192.168.1.1)에는 붙지 않는다. 시뮬에서는 rally_robot gripper_node 대신 이것을 띄운다 (scripts/sim_robot.sh 가 띄움).

    python3 scripts/sim_gripper.py
    ros2 service call /rally/sim/give_ball std_srvs/srv/Trigger      # 손에 공 쥐여 주기 (실로봇에서 손으로 공을 대고 닫는 것과 같음)
    ros2 service call /rally/sim/place_ball std_srvs/srv/Trigger     # 공을 레인 2 입구에 놓기 (빈손으로 밀기 시험)

모드 5 흉내 (막대 + 환자):
    막대(지름 --rod-mm, 길이 --rod-len)가 --rod x,y,yaw 에 놓여 있다. 쥐면 툴 y 축(손가락이 닫히는 툴 x 와 수직)을 따라 붙는다.
    환자: 로봇이 제시 위치(한계선 12 cm 안)에서 --patient-s 초 멈춰 있으면
      · 막대를 쥔 채면(GIVE) +x 로 --patient-n 만큼 당긴다 → 그리퍼가 열리면 막대를 받아 든다
      · 환자가 막대를 든 채 로봇이 물러났다 빈손(열림)으로 다시 오면(RECEIVE) 막대를 그리퍼 사이에 밀어 넣으며 −x 로 민다
    힘은 /rally/sim/external_force(Vector3, N, base)로 발행 → skill_server(RALLY_SIM=1)가 get_tool_force 에 더한다.
    RViz: 막대(초록 원기둥) · 환자 손(살구색) · 카메라 시야(손이 보이면 초록, 안 보이면 회색 사각뿔).
모드 1 환자 흉내 (--mode1-patient, 10/8 회의 흐름): /rally/perception/hand(HandState) 를 15 Hz 발행. 손은 무작위 레인 앞
    (중앙선 + 0.28 m, 안전감시 SLOW 밖)에서 기다리다, 친 공이 레인 끝(x > 0.6)에 멈추면 1 s 뒤 손을 공으로 → 쥠(grip 0.9) 1.5 s →
    폄(0) + 공을 로봇이 닿는 곳(레인 x 0.50)으로 굴려 줌 → 손 제자리 → 다음 레인 무작위. 거치대에 놓인 공(레인 밖)은 굴러 내려가 맨 앞으로.
가상 비전: /rally/perception/objects (base_link) 로 공·막대 위치를 10 Hz 발행 — 모드 1 비전 집기(PushBall pickup_source: vision)를 시뮬에서.
    공은 처음에 거치대(--rack x,y)에 놓여 있다. skills.yaml mode1.vision_offset_m 보정을 거꾸로 빼서 발행 → 로봇이 보정 후 정확히 공 위로 간다.

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
from geometry_msgs.msg import Point, Vector3
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
import random

from rally_interfaces.msg import HandState, TrackedObject, TrackedObjectArray
from rally_interfaces.srv import GripperCommand

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "ros2_ws/src/rally_bringup/config"
TCP_OFFSET = (0.0008, -0.0014, 0.2259)        # link_6 프레임 기준 TCP (print_pose.py 와 같음)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def quat_z_to(v):
    """마커 z 축(원기둥 축)을 v 방향으로 돌리는 쿼터니언."""
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    x, y, z = (c / n for c in v)
    if z < -0.9999:
        return (1.0, 0.0, 0.0, 0.0)
    q = (-y, x, 0.0, 1.0 + z)
    m = math.sqrt(sum(c * c for c in q))
    return tuple(c / m for c in q)


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # v' = q v q*
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


class SimGripper(Node):
    def __init__(self, ball_mm: float, decel: float, rod=(0.30, 0.25, 0.0), rod_mm=30.0, rod_len=0.20,
                 patient_s=2.0, patient_n=12.0, rack=(0.334, 0.282), refill=False, patient1=False):
        super().__init__("rally_sim_gripper")
        skills = yaml.safe_load((CFG / "skills.yaml").read_text(encoding="utf-8"))
        table = yaml.safe_load((CFG / "table.yaml").read_text(encoding="utf-8"))["table"]
        self.lanes = skills["mode1"]["lanes"]
        self.table_z = float(table.get("z_m") or 0.0)
        self.r = ball_mm / 2000.0
        self.decel = decel
        self.width = 110.0              # 현재 손가락 폭 mm
        self.target = -1.0              # 마지막 명령 폭 mm
        self.held = False
        self.ball = [rack[0], rack[1], self.table_z + self.r] if rack else None   # [x, y, z] base_link — 없으면 화면에 안 그림. 처음엔 거치대 위
        self.vision_offset = (skills["mode1"].get("vision_offset_m") or [0.0, 0.0, 0.0])
        self.rack, self.refill = rack, refill      # refill: 굴러가 멈춘 공을 2 s 뒤 거치대로 (거치대 공 여러 개 흉내 — brain 모드 1 은 4번 연속)
        self.rest_since = None
        self.patient1 = patient1
        self.patient1_wait_s = 3.0       # 공이 멈추고 잡기까지 — brain 이 손 보기 자세 도착 뒤 "이제 잡으세요" 라고 하면 잡는 흉내 (치고 ≈ 4.6 s 뒤 도착)
        self.p1 = {"state": "idle", "t": 0.0, "lane_y": random.choice([float(ln["start"][1]) for ln in self.lanes]),
                   "grip": 0.0, "catch_y": 0.0}
        self.ball_v = (0.0, 0.0)        # 굴러가는 속도 (m/s)
        self.tip_hist = []              # [(monotonic, x, y)] 최근 0.25 s 손가락 끝 위치
        self.tip_q = (0.0, 0.0, 0.0, 1.0)
        # 모드 3: 패드 거치대 (10/12) — 손잡이 = skills.yaml mode3.pad_rack_grasp_posx 위치. 거기서 닫으면 폭 30 mm 로 쥠, 열면 놓음
        g = ((skills.get("mode3") or {}).get("pad_rack_grasp_posx") or [])
        self.pad_handle = [float(v) / 1000.0 for v in g[:3]] if len(g) >= 3 else None
        self.pad_held, self.pad_mm = False, 30.0
        self.pad_rack0 = list(self.pad_handle) if self.pad_handle else None     # 거치대 그림 자리 (처음 손잡이 위치)
        # 모드 5: 막대 + 환자
        self.centerline_x = float(table.get("centerline_x_m") or 0.568)
        self.rod_r, self.rod_len = rod_mm / 2000.0, rod_len
        self.rod_state = "table"        # table | held | patient
        yaw = math.radians(rod[2])
        tool_y0 = self._tool_down_y(skills)                       # 집는 자세(yaw 0)에서 툴 y 방향 → 막대가 그 방향으로 놓여 있어야 잡힘
        c, s_ = math.cos(yaw), math.sin(yaw)
        self.rod_axis = [tool_y0[0] * c - tool_y0[1] * s_, tool_y0[0] * s_ + tool_y0[1] * c, 0.0]
        self.rod_pos = [rod[0], rod[1], self.table_z + self.rod_r]
        self.patient_s, self.patient_n = patient_s, patient_n
        self.hand_rest = [self.centerline_x + 0.25, 0.0, self.table_z + 0.15]
        self.hand = list(self.hand_rest)
        self.still_ref, self.still_since = None, time.monotonic()
        self.release_spot, self.came_back = None, False
        self.force = (0.0, 0.0, 0.0)
        self.hand_seen = False
        self.offering = False           # 테이블에서 집은 막대를 들고 있음 → 환자가 당길 차례 (돌려받은 막대는 안 당김)
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
        self.pub_force = self.create_publisher(Vector3, "/rally/sim/external_force", 10)
        self.pub_obj = self.create_publisher(TrackedObjectArray, "/rally/perception/objects", qos_profile_sensor_data)
        self.create_timer(0.1, self.publish_objects, callback_group=cbg)
        if patient1:
            self.pub_hand = self.create_publisher(HandState, "/rally/perception/hand", qos_profile_sensor_data)
            self.create_timer(1.0 / 15.0, self.publish_hand, callback_group=cbg)
        self.create_timer(0.02, self.step, callback_group=cbg)
        self.create_timer(0.05, self.publish_state, callback_group=cbg)
        self.get_logger().info(f"sim gripper ready — 공 지름 {ball_mm:.0f} mm, 감속도 {decel} m/s² · /rally/sim/give_ball · /rally/sim/place_ball")

    @staticmethod
    def _tool_down_y(skills):
        zyz = (skills.get("pick") or {}).get("tool_down_zyz") or [0.0, 180.0, 0.0]
        a, b, c = (math.radians(v) for v in zyz)
        # R = Rz(a) Ry(b) Rz(c) 의 둘째 열 (툴 y 축)
        y = (-math.cos(a) * math.cos(b) * math.sin(c) - math.sin(a) * math.cos(c),
             -math.sin(a) * math.cos(b) * math.sin(c) + math.cos(a) * math.cos(c))
        n = math.hypot(*y) or 1.0
        return (y[0] / n, y[1] / n)

    # ----- 그리퍼 명령 -----
    def on_cmd(self, req, res):
        target = min(max(float(req.width_mm), 0.0), 110.0)
        with self.lock:
            self.target = target
            ball_mm = self.r * 2000.0
            rod_mm = self.rod_r * 2000.0
            tip = self._tip()
            if self.rod_state == "held" and target >= rod_mm:          # 막대 놓기
                if tip is not None and tip[3] < self.table_z + 0.06:  # 테이블 위 → 눕힘
                    self.rod_state = "table"
                    self.rod_pos[2] = self.table_z + self.rod_r
                    h = math.hypot(self.rod_axis[0], self.rod_axis[1]) or 1.0
                    self.rod_axis = [self.rod_axis[0] / h, self.rod_axis[1] / h, 0.0]
                else:                                                  # 공중(제시 위치) → 환자가 받아 듦
                    self.rod_state = "patient"
                    self.release_spot, self.came_back = (tip[1:] if tip else None), False
                    self.get_logger().info("환자가 막대를 받음")
            if (self.rod_state != "held" and not self.held and target < rod_mm and tip is not None
                    and math.dist(tip[1:], self.rod_pos) < 0.03):
                self.offering = self.rod_state == "table"
                self.rod_state = "held"
                self.get_logger().info("막대 쥠")
            if self.pad_held and target >= 40.0:                       # 패드 놓기 (그 자리에 남음)
                self.pad_held = False
                if tip is not None:
                    self.pad_handle = list(tip[1:])
                self.get_logger().info("패드 놓음")
            if (not self.pad_held and not self.held and self.rod_state != "held" and target < self.pad_mm
                    and tip is not None and self.pad_handle is not None and math.dist(tip[1:], self.pad_handle) < 0.03):
                self.pad_held = True
                self.get_logger().info(f"패드 쥠 (손잡이와 {math.dist(tip[1:], self.pad_handle) * 1000:.0f} mm)")
            if self.held and target >= ball_mm:          # 열기 → 놓기
                self.held = False
                self.ball[2] = self.table_z + self.r
                self.ball_v = (0.0, 0.0)
            grab = (not self.held) and self.rod_state != "held" and not self.pad_held and target < ball_mm and self._ball_between_fingers()
            if grab:
                self.held = True
            if self.pad_held:
                actual = self.pad_mm
            elif self.rod_state == "held":
                actual = rod_mm
            else:
                actual = ball_mm if self.held else max(target, 12.4)   # 실물 RG2: 빈손으로 닫아도 12.4 mm
            self.width = actual
            holding = self.held or self.rod_state == "held" or self.pad_held
        if self.cli.wait_for_service(timeout_sec=0.5):
            ev = threading.Event()
            fut = self.cli.call_async(SetCommand.Request(command=str(int(actual * 10))))
            fut.add_done_callback(lambda _f: ev.set())
            ev.wait(5.0 if req.wait_until_done else 0.0)
        res.success, res.actual_width_mm, res.object_detected = True, float(actual), bool(holding)
        self.get_logger().info(f"width {target:.0f} → {actual:.0f} mm, 쥠 {holding}")
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
        self.tip_q = (q.x, q.y, q.z, q.w)
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
            if tip is not None and self.pad_held:
                self.pad_handle = list(tip[1:])
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
                touching = tip[3] < self.table_z + 1.5 * self.r and math.hypot(tip[1] - bx, tip[2] - by) < self.r + 0.012   # 손가락 끝이 공 중심 근처 높이일 때만 민다
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
            if self.refill and self.rack and self.ball is not None and not self.held:
                away = math.hypot(self.ball[0] - self.rack[0], self.ball[1] - self.rack[1]) > 0.05
                if self.patient1 and self._lane_of(self.ball) is not None:
                    away = False                     # 레인 안 공은 환자·ReturnBall 몫 — 거치대에 놓인 공(레인 밖)만 굴러 내려감
                if away and math.hypot(*self.ball_v) < 1e-4 and (tip is None or math.dist(tip[1:], self.ball) > 0.08):
                    self.rest_since = self.rest_since or time.monotonic()
                    if time.monotonic() - self.rest_since > 2.0:
                        self.ball, self.ball_v, self.rest_since = [self.rack[0], self.rack[1], self.table_z + self.r], (0.0, 0.0), None
                        self.get_logger().info("멈춘 공을 거치대로 되돌림 (--refill)")
                else:
                    self.rest_since = None
            ball = list(self.ball) if self.ball else None
            self._patient(tip)
            if self.patient1:
                self._patient1()
        f = (float(self.force[0]), float(self.force[1]), float(self.force[2]))
        if f != getattr(self, "_last_force", None):        # 바뀔 때만 — 다른 시험 스크립트가 넣는 힘(모드 7 밀기 등)을 0 으로 덮지 않게 (10/12)
            self._last_force = f
            self.pub_force.publish(Vector3(x=f[0], y=f[1], z=f[2]))
        self.publish_scene(ball)

    def _patient(self, tip):
        """막대 따라붙기 + 환자 흉내 (lock 안에서 호출)."""
        now = time.monotonic()
        self.force = (0.0, 0.0, 0.0)
        if tip is None:
            return
        p = tip[1:]
        if self.still_ref is None or math.dist(p, self.still_ref) > 0.002:
            self.still_ref, self.still_since = p, now
        still = now - self.still_since
        at_present = p[0] > self.centerline_x - 0.12
        reach = [p[0] + 0.06, p[1], p[2]]                       # 환자 손이 잡는 자리 (손끝 바로 앞)
        if self.rod_state == "held":
            self.rod_pos = list(p)
            self.rod_axis = list(quat_rotate(self.tip_q, (0.0, 1.0, 0.0)))   # 막대 = 툴 y 축
            if self.offering and at_present and still >= self.patient_s:
                self.hand = reach
                self.force = (self.patient_n, 0.0, 0.0)          # 환자가 잡고 당김
            else:
                self.hand = list(self.hand_rest)
        elif self.rod_state == "patient":
            if self.release_spot is not None and math.dist(p, self.release_spot) > 0.05:
                self.came_back = True                            # 로봇이 물러났다가
            if self.came_back and at_present and still >= self.patient_s and self.target >= self.rod_r * 2000.0:
                self.hand = reach                                # 빈손(열림)으로 다시 오면 그리퍼 사이로 밀어 넣음
                self.rod_pos = list(p)
                self.rod_axis = list(quat_rotate(self.tip_q, (0.0, 1.0, 0.0)))
                self.force = (-self.patient_n, 0.0, 0.0)
            elif not self.came_back:
                self.hand = reach if self.release_spot else list(self.hand_rest)
                self.rod_pos = list(self.hand)
            else:                                                # 환자가 막대를 들고 기다림 (세로로)
                self.hand = [self.centerline_x + 0.12, 0.0, self.table_z + 0.22]
                self.rod_pos, self.rod_axis = list(self.hand), [0.0, 0.0, 1.0]
        else:
            self.hand = list(self.hand_rest)

    def _lane_of(self, p):
        for ln in self.lanes:
            if abs(p[1] - float(ln["start"][1])) < 0.07 and p[0] > float(ln["start"][0]) - 0.05:
                return float(ln["start"][1])
        return None

    def _patient1(self):
        """모드 1 환자 (lock 안). 손 = self.hand, 쥔 정도 = self.p1['grip']."""
        p1, now, cl = self.p1, time.monotonic(), self.centerline_x
        rest = [cl + 0.28, p1["lane_y"], self.table_z + 0.12]
        b = self.ball
        resting = b is not None and not self.held and math.hypot(*self.ball_v) < 1e-4
        if p1["state"] == "idle":
            self.hand, p1["grip"] = rest, 0.0
            ly = self._lane_of(b) if resting else None
            if ly is not None and b[0] > 0.6:
                p1.update(state="reach", t=now, catch_y=ly)
        elif p1["state"] == "reach":
            if now - p1["t"] > self.patient1_wait_s:
                self.hand = [b[0] + 0.04, p1["catch_y"], self.table_z + 0.06]
                p1.update(state="hold", t=now, grip=0.9)
                self.get_logger().info("환자: 공을 쥠")
        elif p1["state"] == "hold":
            self.ball = [self.hand[0] - 0.04, self.hand[1], self.table_z + self.r]     # 손에 든 공
            if now - p1["t"] > 1.5:
                self.ball, self.ball_v = [0.50, p1["catch_y"], self.table_z + self.r], (0.0, 0.0)   # 로봇 쪽으로 굴려 줌
                p1.update(state="idle", t=now, grip=0.0,
                          lane_y=random.choice([float(ln["start"][1]) for ln in self.lanes]))
                self.get_logger().info("환자: 공을 놓고 로봇 쪽으로 굴려 줌")

    def publish_hand(self):
        m = HandState()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "base_link"
        with self.lock:
            hx, hy, hz = self.hand
            grip = self.p1["grip"]
        m.detected, m.handedness = True, "right"
        m.landmarks = [Point(x=hx, y=hy, z=hz) for _ in range(21)]
        m.palm_pose.position = Point(x=hx, y=hy, z=hz)
        m.palm_pose.orientation.w = 1.0
        m.grip_closure = float(grip)
        m.gesture = HandState.GESTURE_NONE
        m.distance_to_centerline = float(hx - self.centerline_x)
        self.pub_hand.publish(m)

    def _camera(self):
        try:
            tr = self.tf.lookup_transform("base_link", "camera_color_optical_frame", Time(), timeout=Duration(seconds=0.02))
        except Exception:                                   # noqa: BLE001
            return None
        q = tr.transform.rotation
        o = tr.transform.translation
        return (o.x, o.y, o.z), (q.x, q.y, q.z, q.w)

    # ----- 발행 -----
    def publish_state(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ["rg2_width_mm", "rg2_grip", "rg2_target_mm"]
        with self.lock:
            m.position = [float(self.width), 1.0 if (self.held or self.rod_state == "held") else 0.0, float(self.target)]
        self.pub_state.publish(m)

    def publish_objects(self):
        m = TrackedObjectArray()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "base_link"
        ox, oy, oz = (float(v) for v in self.vision_offset)
        with self.lock:
            items = []
            if self.ball is not None and not self.held:
                items.append(("ball", 1, self.ball, math.hypot(*self.ball_v) > 0.01))
            if self.rod_state == "table":
                items.append(("rod", 2, self.rod_pos, False))
            yaw = math.atan2(self.rod_axis[1], self.rod_axis[0])
        for cls, tid, pos, moving in items:
            o = TrackedObject()
            o.header = m.header
            o.track_id, o.class_name, o.confidence, o.is_moving = tid, cls, 0.9, moving
            o.position = Point(x=pos[0] - ox, y=pos[1] - oy, z=pos[2] - oz)
            o.yaw = yaw if cls == "rod" else 0.0
            m.objects.append(o)
        self.pub_obj.publish(m)

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
        with self.lock:
            rod_pos, rod_axis, hand = list(self.rod_pos), list(self.rod_axis), list(self.hand)
        m = mk(200, Marker.CYLINDER, tuple(rod_pos), (2 * self.rod_r, 2 * self.rod_r, self.rod_len), (0.2, 0.7, 0.3, 1.0), "rod")
        qx, qy, qz, qw = quat_z_to(rod_axis)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qx, qy, qz, qw
        arr.markers.append(m)
        if self.pad_rack0 is not None:          # 모드 3 패드 거치대·패드 (모양은 대략 — 손잡이 아래 벨크로 면, 그 아래 거치대)
            rx, ry, rz = self.pad_rack0
            arr.markers.append(mk(500, Marker.CUBE, (rx, ry, rz - 0.21), (0.09, 0.09, 0.08), (0.45, 0.45, 0.5, 0.8), "pad"))
            with self.lock:
                hx, hy, hz = self.pad_handle
            arr.markers.append(mk(501, Marker.CUBE, (hx, hy, hz + 0.01), (0.025, 0.025, 0.06), (0.15, 0.15, 0.15, 1.0), "pad"))
            disk = mk(502, Marker.CYLINDER, (hx, hy, hz - 0.10), (0.15, 0.15, 0.012), (0.85, 0.2, 0.2, 0.9), "pad")   # 패드는 손잡이 아래
            disk.pose.orientation.x, disk.pose.orientation.w = math.sqrt(0.5), math.sqrt(0.5)    # 면을 세움
            arr.markers.append(disk)
        arr.markers.append(mk(300, Marker.CUBE, tuple(hand), (0.10, 0.08, 0.04), (0.96, 0.75, 0.6, 1.0), "hand"))
        arr.markers.append(mk(301, Marker.CUBE, (hand[0] + 0.17, hand[1], hand[2]), (0.24, 0.06, 0.06), (0.96, 0.75, 0.6, 0.6), "hand"))
        cam = self._camera()
        if cam is not None:
            o, q = cam
            d, tx, ty = 0.7, math.tan(math.radians(34.5)), math.tan(math.radians(21.0))     # D435 컬러 시야 69° × 42°
            corners = [quat_rotate(q, (sx * tx * d, sy * ty * d, d)) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
            corners = [(o[0] + c[0], o[1] + c[1], o[2] + c[2]) for c in corners]
            # 손이 시야 안인지 (카메라 좌표로 바꿔 각도 비교)
            inv = (-q[0], -q[1], -q[2], q[3])
            hx, hy, hz = quat_rotate(inv, (hand[0] - o[0], hand[1] - o[1], hand[2] - o[2]))
            seen = 0.05 < hz < 1.2 and abs(hx) < tx * hz and abs(hy) < ty * hz
            if seen != self.hand_seen:
                self.hand_seen = seen
                self.get_logger().info(f"카메라 시야에 환자 손 {'보임' if seen else '안 보임'}")
            f = mk(400, Marker.LINE_LIST, (0.0, 0.0, 0.0), (0.004, 0.0, 0.0), (0.1, 0.8, 0.2, 0.9) if seen else (0.6, 0.6, 0.6, 0.6), "camera")
            for i, c in enumerate(corners):
                f.points += [Point(x=o[0], y=o[1], z=o[2]), Point(x=c[0], y=c[1], z=c[2])]
                n = corners[(i + 1) % 4]
                f.points += [Point(x=c[0], y=c[1], z=c[2]), Point(x=n[0], y=n[1], z=n[2])]
            arr.markers.append(f)
        self.pub_scene.publish(arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ball-mm", type=float, default=63.0)
    ap.add_argument("--decel", type=float, default=0.08, help="공 감속도 [m/s²] (레시피 실험 전 추정값)")
    ap.add_argument("--object-mm", type=float, default=None, help="(이전 옵션 호환) --ball-mm 과 같음")
    ap.add_argument("--rod", default="0.30,-0.25,0", help="막대 위치 x,y[m],yaw[deg] (모드 5)")
    ap.add_argument("--mode1-patient", action="store_true", help="모드 1 환자 흉내 (손 인식 발행·공 받기·굴려 주기). --refill 과 같이")
    ap.add_argument("--refill", action="store_true", help="굴러가 멈춘 공을 2 s 뒤 거치대로 되돌림 (brain 모드 1 연속 시도용)")
    ap.add_argument("--rack", default="0.334,0.282", help="처음 공 위치 = 거치대 x,y[m] (모드 1 비전 집기). none 이면 공 없음")
    ap.add_argument("--rod-mm", type=float, default=30.0)
    ap.add_argument("--rod-len", type=float, default=0.20)
    ap.add_argument("--patient-s", type=float, default=2.0, help="제시 위치에서 이 시간 멈추면 환자가 당김/밀어 넣음")
    ap.add_argument("--patient-n", type=float, default=12.0)
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    rclpy.init()
    rod = tuple(float(v) for v in opt.rod.split(","))
    rack = None if opt.rack == "none" else tuple(float(v) for v in opt.rack.split(","))
    n = SimGripper(opt.object_mm or opt.ball_mm, opt.decel, rod, opt.rod_mm, opt.rod_len, opt.patient_s, opt.patient_n, rack, opt.refill, opt.mode1_patient)
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
