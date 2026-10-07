#!/usr/bin/env python3
"""관절 외력 토크(EJT) → 자코비안 → 툴 외력 추정 (get_tool_force 가 0 으로만 올 때의 우회, 2026-10-07).

소스: realtime/read_data_rt (RT 스트림, 2.5 ms 갱신·왕복 0.4 ms). 관절각(actual_joint_position)과 외력 토크(external_joint_torque)를
같은 패킷에서 꺼낸다. aux_control/get_external_torque 는 모니터링 캐시라 10 Hz 로만 갱신되므로 쓰지 않는다 (--source mon 으로 비교 가능).

    source scripts/env_robot_pc.sh && python3 scripts/ejt_wrench.py --seconds 15 [--rate 5]
    python3 scripts/ejt_wrench.py --publish --rate 50 --seconds 600     # force_publisher 대신 /rally/robot/wrench 발행 (measure_impact/measure_wrench 용)

컨트롤러의 get_external_torque(τ_ext, N·m)와 /dsr01/joint_states(q)로 F = (Jᵀ)⁻¹ τ_ext 를 푼다. 힘은 base 기준·플랜지 작용점.
검증: ① 정지 시 |F| 가 몇 N 이내 ② FK 플랜지 위치가 get_current_posx(TCP) 와 TCP 오프셋만큼만 다름 ③ 그리퍼를 손으로 아래로 누르면 Fz<0 (크기 ≈ 누른 힘).
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from dsr_msgs2.srv import GetExternalTorque, GetCurrentPosx, ReadDataRt
from urdf_parser_py.urdf import URDF
from ament_index_python.packages import get_package_share_directory

JOINTS = [f"joint_{i}" for i in range(1, 7)]


def _snap(a):  # URDF 의 1.571 같은 반올림값을 π/2 배수로
    k = round(a / (math.pi / 2))
    return k * math.pi / 2 if abs(a - k * math.pi / 2) < 2e-3 else a


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _rotz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class Chain:
    """base_link → link_6 (6 revolute, 축은 모두 로컬 z)."""
    def __init__(self):
        urdf = URDF.from_xml_file(get_package_share_directory("dsr_description2") + "/urdf/m0609.urdf")
        by_name = {j.name: j for j in urdf.joints}
        self.T0 = []
        for n in JOINTS:
            j = by_name[n]
            assert j.type == "revolute" and list(j.axis) == [0.0, 0.0, 1.0], n
            T = np.eye(4)
            T[:3, :3] = _rpy(*[_snap(a) for a in j.origin.rpy])
            T[:3, 3] = j.origin.xyz
            self.T0.append(T)

    def fk_frames(self, q):
        T = np.eye(4)
        frames = []
        for T0, qi in zip(self.T0, q):
            T = T @ T0
            frames.append(T.copy())          # 관절 i 의 프레임(회전 전): 축 z_i, 원점 o_i
            R = np.eye(4); R[:3, :3] = _rotz(qi)
            T = T @ R
        return frames, T                      # T = 플랜지(link_6) 자세

    def jacobian(self, q):
        frames, Tf = self.fk_frames(q)
        p = Tf[:3, 3]
        J = np.zeros((6, 6))
        for i, F in enumerate(frames):
            z, o = F[:3, 2], F[:3, 3]
            J[:3, i] = np.cross(z, p - o)
            J[3:, i] = z
        return J, Tf


class EjtWrench(Node):
    def __init__(self):
        super().__init__("rally_ejt_wrench")
        self.q = None
        self.damping = 0.05
        self.chain = Chain()
        cbg = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/dsr01/joint_states", self._js, qos_profile_sensor_data, callback_group=cbg)
        self.cli = self.create_client(GetExternalTorque, "/dsr01/dsr_controller2/aux_control/get_external_torque", callback_group=cbg)
        self.rt = self.create_client(ReadDataRt, "/dsr01/dsr_controller2/realtime/read_data_rt", callback_group=cbg)
        self.source = "rt"
        self.posx = self.create_client(GetCurrentPosx, "/dsr01/dsr_controller2/aux_control/get_current_posx", callback_group=cbg)
        self.pub = self.create_publisher(WrenchStamped, "/rally/robot/wrench", qos_profile_sensor_data)

    def publish(self, F):
        m = WrenchStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = float(F[0]), float(F[1]), float(F[2])
        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = float(F[3]), float(F[4]), float(F[5])
        self.pub.publish(m)

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in JOINTS):
            self.q = np.array([d[n] for n in JOINTS])

    def sample(self):
        if self.source == "rt":
            fut = self.rt.call_async(ReadDataRt.Request())
        else:
            fut = self.cli.call_async(GetExternalTorque.Request())
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(2.0):
            return None
        if self.source == "rt":
            d = fut.result().data
            q = np.radians(np.array(d.actual_joint_position, dtype=float))
            tau = np.array(d.external_joint_torque, dtype=float)
        else:
            if self.q is None:
                return None
            q, tau = self.q, np.array(fut.result().ext_torque, dtype=float)
        J, Tf = self.chain.jacobian(q)
        cond = np.linalg.cond(J.T)
        # 감쇠 최소자승: τ = Jᵀ F  →  F = (J Jᵀ + λ²I)⁻¹ J τ. 특이점 근처(최소 특이값 ≪ λ)에서 노이즈 증폭을 λ 로 제한
        lam = self.damping
        F = np.linalg.solve(J @ J.T + lam * lam * np.eye(6), J @ tau)
        return F, tau, Tf, cond


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--source", choices=["rt", "mon"], default="rt", help="rt=read_data_rt(2.5 ms 갱신) · mon=get_external_torque(10 Hz 캐시)")
    ap.add_argument("--publish", action="store_true", help="/rally/robot/wrench 로 발행 (force_publisher 대체). 화면 출력은 1 Hz")
    ap.add_argument("--damping", type=float, default=0.05, help="DLS λ (m 단위 특이값 기준, 0.02~0.1)")
    opt = ap.parse_args()
    rclpy.init()
    n = EjtWrench()
    n.damping = opt.damping
    n.source = opt.source
    ex = MultiThreadedExecutor(2)
    ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()
    n.cli.wait_for_service(5.0); n.posx.wait_for_service(5.0); n.rt.wait_for_service(5.0)
    t0 = time.monotonic()
    while n.q is None and time.monotonic() - t0 < 5.0:
        time.sleep(0.05)
    if n.q is None:
        print("/dsr01/joint_states 수신 없음"); return 1
    fut = n.posx.call_async(GetCurrentPosx.Request(ref=0))
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < 3.0:
        time.sleep(0.01)
    _, Tf = n.chain.fk_frames(n.q)
    print(f"FK 플랜지 위치(base, mm): {np.round(Tf[:3, 3] * 1000, 1)}  플랜지 z축(base): {np.round(Tf[:3, 2], 3)}")
    if fut.done():
        px = fut.result().task_pos_info[0].data
        print(f"컨트롤러 posx(TCP, mm): {np.round(px[:3], 1)}  차이(mm): {np.round(np.array(px[:3]) - Tf[:3, 3] * 1000, 1)}  ← TCP 오프셋(플랜지→TCP)이어야 함")
    print(f"\n{opt.seconds:.0f} s 동안 {opt.rate:.0f} Hz 출력. 중간에 그리퍼를 손으로 아래로 눌러 보세요 (Fz 가 음수로 커져야 함)")
    print("   t     Fx     Fy     Fz    |F|   |τ_ext|  cond")
    t0 = time.monotonic()
    mags = []
    period, next_t, last_print, miss = 1.0 / opt.rate, time.monotonic(), 0.0, 0
    try:
        while time.monotonic() - t0 < opt.seconds:
            s = n.sample()
            now = time.monotonic()
            if s:
                F, tau, _, cond = s
                mags.append(np.linalg.norm(F[:3]))
                if opt.publish:
                    n.publish(F)
                if not opt.publish or now - last_print >= 1.0:
                    last_print = now
                    print(f"{now-t0:5.1f} {F[0]:6.2f} {F[1]:6.2f} {F[2]:6.2f} {mags[-1]:6.2f}  {np.linalg.norm(tau):6.2f}  {cond:5.0f}"
                          + ("  ⚠ 특이점 근처(힘 과대)" if cond > 100 else "") + (f"  (서비스 미응답 {miss}회)" if miss else ""))
            else:
                miss += 1
            next_t += period
            if next_t < time.monotonic() - period:      # 멈칫한 뒤 밀린 틱을 몰아서 쏘지 않는다
                next_t = time.monotonic()
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        pass
    if mags:
        print(f"\n|F| 평균 {np.mean(mags):.2f} N · 최대 {np.max(mags):.2f} N · 샘플 {len(mags)}개 ({len(mags)/(time.monotonic()-t0):.0f} Hz 달성, 목표 {opt.rate:.0f}) · 서비스 미응답 {miss}회")
    ex.shutdown(); n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
