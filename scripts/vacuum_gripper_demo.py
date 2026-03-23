#!/usr/bin/env python3
"""
Demo script for the franka_vacuum_gripper ROS2 driver.

Demonstrates a pick-and-place cycle:
  1. Activate vacuum to pick an object
  2. Wait for confirmation (part_present / in_control_range)
  3. Drop the object off
  4. Stop the gripper

Usage:
    ros2 run franka_vacuum_gripper vacuum_gripper_demo.py
"""

import sys
import time

import rclpy
from rclpy.node import Node

from franka_vacuum_gripper.msg import VacuumGripperState
from franka_vacuum_gripper.srv import DropOff, Vacuum
from std_srvs.srv import Trigger

DEVICE_STATUS_LABELS = {
    VacuumGripperState.DEVICE_STATUS_GREEN: "GREEN",
    VacuumGripperState.DEVICE_STATUS_YELLOW: "YELLOW",
    VacuumGripperState.DEVICE_STATUS_ORANGE: "ORANGE",
    VacuumGripperState.DEVICE_STATUS_RED: "RED",
}


class VacuumGripperDemo(Node):
    def __init__(self):
        super().__init__("vacuum_gripper_demo")

        self._state: VacuumGripperState | None = None

        # Subscribe to gripper state
        self._state_sub = self.create_subscription(
            VacuumGripperState,
            "/franka_vacuum_gripper/state",
            self._state_callback,
            10,
        )

        # Service clients
        self._vacuum_client = self.create_client(Vacuum, "/franka_vacuum_gripper/vacuum")
        self._drop_off_client = self.create_client(DropOff, "/franka_vacuum_gripper/drop_off")
        self._stop_client = self.create_client(Trigger, "/franka_vacuum_gripper/stop")

        self.get_logger().info("Waiting for vacuum gripper services...")
        self._vacuum_client.wait_for_service()
        self._drop_off_client.wait_for_service()
        self._stop_client.wait_for_service()
        self.get_logger().info("Services ready.")

    def _state_callback(self, msg: VacuumGripperState):
        self._state = msg

    def print_state(self):
        if self._state is None:
            self.get_logger().warn("No state received yet.")
            return
        s = self._state
        status_label = DEVICE_STATUS_LABELS.get(s.device_status, "UNKNOWN")
        self.get_logger().info(
            f"State | vacuum={s.vacuum} mbar  power={s.actual_power}%  "
            f"part_present={s.part_present}  in_control_range={s.in_control_range}  "
            f"part_detached={s.part_detached}  device={status_label}"
        )

    def vacuum(self, setpoint: int = 60, timeout_ms: int = 3000, profile: str = "P0") -> bool:
        """Activate vacuum. Returns True if vacuum was established."""
        req = Vacuum.Request()
        req.vacuum = setpoint
        req.timeout_ms = timeout_ms
        req.profile = profile

        self.get_logger().info(
            f"Requesting vacuum: setpoint={setpoint} mbar, timeout={timeout_ms} ms, profile={profile}"
        )
        future = self._vacuum_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result: Vacuum.Response = future.result()
        self.get_logger().info(f"Vacuum result: success={result.success}, msg='{result.message}'")
        return result.success

    def drop_off(self, timeout_ms: int = 3000) -> bool:
        """Drop off the currently held object. Returns True on success."""
        req = DropOff.Request()
        req.timeout_ms = timeout_ms

        self.get_logger().info(f"Requesting drop-off: timeout={timeout_ms} ms")
        future = self._drop_off_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result: DropOff.Response = future.result()
        self.get_logger().info(f"Drop-off result: success={result.success}, msg='{result.message}'")
        return result.success

    def stop(self) -> bool:
        """Stop any running vacuum or drop-off operation."""
        req = Trigger.Request()
        self.get_logger().info("Requesting stop")
        future = self._stop_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result: Trigger.Response = future.result()
        self.get_logger().info(f"Stop result: success={result.success}, msg='{result.message}'")
        return result.success

    def wait_for_state(self, timeout_s: float = 2.0):
        """Spin briefly to receive an updated state message."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and self._state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        rclpy.spin_once(self, timeout_sec=0.1)  # one extra tick for freshness


def main():
    rclpy.init()
    demo = VacuumGripperDemo()

    try:
        # --- 1. Read initial state ---
        demo.get_logger().info("=== Reading initial gripper state ===")
        demo.wait_for_state()
        demo.print_state()
        time.sleep(0.5)

        # --- 2. Activate vacuum ---
        demo.get_logger().info("=== Activating vacuum (pick) ===")
        success = demo.vacuum(setpoint=60, timeout_ms=5000, profile="P0")
        if not success:
            demo.get_logger().warn("Vacuum not established — is an object present?")

        demo.wait_for_state()
        demo.print_state()
        time.sleep(1.0)

        # --- 3. Drop off ---
        demo.get_logger().info("=== Dropping off object ===")
        success = demo.drop_off(timeout_ms=3000)
        if not success:
            demo.get_logger().warn("Drop-off may have failed — check gripper state")

        demo.wait_for_state()
        demo.print_state()
        time.sleep(0.5)

        # --- 4. Stop (clean up) ---
        demo.get_logger().info("=== Stopping gripper ===")
        demo.stop()

        demo.wait_for_state()
        demo.print_state()

    except KeyboardInterrupt:
        demo.get_logger().info("Interrupted — stopping gripper")
        demo.stop()
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
