#!/usr/bin/env python3
"""시뮬 전용 그리퍼 — /rally/gripper/command 를 받아 가상 RG2(/onrobot/sendCommand, bringup mode:=virtual)로 넘긴다.
실제 그리퍼(Modbus 192.168.1.1)에는 붙지 않는다. 시뮬에서는 rally_robot gripper_node 대신 이것을 띄운다.

    python3 scripts/sim_gripper.py                 # 닫으면 물체 없음(폭 0)
    python3 scripts/sim_gripper.py --object-mm 60  # 지름 60 mm 물체를 쥔 것처럼 (폭 60 에서 멈추고 object_detected)
"""
import argparse
import sys
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from onrobot_rg_msgs.srv import SetCommand
from rally_interfaces.srv import GripperCommand


class SimGripper(Node):
    def __init__(self, object_mm: float):
        super().__init__("rally_sim_gripper")
        self.object_mm = object_mm
        cbg = ReentrantCallbackGroup()
        self.cli = self.create_client(SetCommand, "/onrobot/sendCommand", callback_group=cbg)
        self.create_service(GripperCommand, "/rally/gripper/command", self.on_cmd, callback_group=cbg)
        self.get_logger().info(f"sim gripper ready (가상 RG2 로 전달, 물체 {object_mm:.0f} mm)")

    def on_cmd(self, req, res):
        target = min(max(float(req.width_mm), 0.0), 110.0)
        holding = self.object_mm > 0 and target < self.object_mm
        actual = self.object_mm if holding else target
        if self.cli.wait_for_service(timeout_sec=1.0):
            ev = threading.Event()
            fut = self.cli.call_async(SetCommand.Request(command=str(int(actual * 10))))
            fut.add_done_callback(lambda _f: ev.set())
            ev.wait(5.0 if req.wait_until_done else 0.0)
        else:
            self.get_logger().warning("/onrobot/sendCommand 없음 — RViz 손가락은 안 움직이고 응답만 돌려줌")
        res.success, res.actual_width_mm, res.object_detected = True, float(actual), bool(holding)
        self.get_logger().info(f"width {target:.0f} → {actual:.0f} mm, object {holding}")
        return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object-mm", type=float, default=0.0)
    opt = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])
    rclpy.init()
    n = SimGripper(opt.object_mm)
    ex = MultiThreadedExecutor(3)
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
