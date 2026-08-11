#!/usr/bin/env python3

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ACTIVE_FAULT_CASES = {
    "localization_lost",
    "lidar_dropout",
    "imu_dropout",
}


def spin_until(node: Node, predicate, timeout_s: float) -> bool:
    """Bound service waits by wall time while continuing to process ROS callbacks."""
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return bool(predicate())


class FailureInjector(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11d_fault_injector")
        self.fault_case = str(
            self.declare_parameter("fault_case", "localization_lost").value
        )
        self.trigger_delay_s = float(
            self.declare_parameter("trigger_delay_s", 9.0).value
        )
        self.service_timeout_s = float(
            self.declare_parameter("service_timeout_s", 5.0).value
        )
        if self.fault_case not in ACTIVE_FAULT_CASES:
            raise RuntimeError(
                f"v25_11d_fault_injector unsupported fault_case={self.fault_case!r}"
            )
        if self.trigger_delay_s <= 0.0 or self.service_timeout_s <= 0.0:
            raise RuntimeError("fault injector timing parameters must be positive")

        self.publisher = self.create_publisher(
            String, "/agt/simulation/fault/state", TRANSIENT_QOS
        )
        self.trigger_client = None
        self.set_bool_client = None
        if self.fault_case == "localization_lost":
            self.service_name = "/agt/simulation/localization/publish_lost"
            self.trigger_client = self.create_client(Trigger, self.service_name)
        elif self.fault_case == "lidar_dropout":
            self.service_name = "/agt/simulation/sensors/set_lidar_drop"
            self.set_bool_client = self.create_client(SetBool, self.service_name)
        else:
            self.service_name = "/agt/simulation/sensors/set_imu_drop"
            self.set_bool_client = self.create_client(SetBool, self.service_name)

        self.trigger_delay_ns = int(self.trigger_delay_s * 1_000_000_000)
        self.started_ros_ns: int | None = None
        self.previous_ros_ns: int | None = None
        self.fired = False
        self.finished = False
        self.state = "ARMED"
        self.message = f"waiting to inject {self.fault_case} on ROS simulation time"
        self._publish_state()

    def _publish_state(self) -> None:
        current_ros_ns = int(self.get_clock().now().nanoseconds)
        msg = String()
        msg.data = json.dumps(
            {
                "gate": "V25-11D",
                "fault_case": self.fault_case,
                "state": self.state,
                "service": self.service_name,
                "trigger_clock": "ros_sim_time",
                "trigger_delay_s": self.trigger_delay_s,
                "started_ros_ns": self.started_ros_ns,
                "current_ros_ns": current_ros_ns,
                "message": self.message,
            },
            separators=(",", ":"),
        )
        self.publisher.publish(msg)

    def ready_to_fire(self) -> bool:
        if self.fired or self.finished:
            return False

        now_ros_ns = int(self.get_clock().now().nanoseconds)
        # use_sim_time nodes report zero until the first /clock sample arrives.
        if now_ros_ns <= 0:
            return False

        if self.started_ros_ns is None:
            self.started_ros_ns = now_ros_ns
            self.previous_ros_ns = now_ros_ns
            self.message = (
                f"armed {self.fault_case} at ROS time {now_ros_ns} ns; "
                f"delay={self.trigger_delay_s:.2f}s"
            )
            self._publish_state()
            return False

        if self.previous_ros_ns is not None and now_ros_ns < self.previous_ros_ns:
            self.state = "FAILED"
            self.finished = True
            self.message = (
                "ROS_TIME_ROLLBACK while waiting to inject fault; "
                f"previous_ns={self.previous_ros_ns} current_ns={now_ros_ns}"
            )
            self._publish_state()
            self.get_logger().error(self.message)
            return False

        self.previous_ros_ns = now_ros_ns
        return now_ros_ns - self.started_ros_ns >= self.trigger_delay_ns

    def fire(self) -> None:
        if self.fired:
            return
        self.fired = True
        self.state = "FIRING"
        self.message = f"calling {self.service_name}"
        self._publish_state()

        client = self.trigger_client if self.trigger_client is not None else self.set_bool_client
        if client is None or not client.wait_for_service(timeout_sec=self.service_timeout_s):
            self.state = "FAILED"
            self.message = f"fault service unavailable: {self.service_name}"
            self.finished = True
            self._publish_state()
            self.get_logger().error(self.message)
            return

        if self.trigger_client is not None:
            future = self.trigger_client.call_async(Trigger.Request())
        else:
            request = SetBool.Request()
            request.data = True
            future = self.set_bool_client.call_async(request)

        if not spin_until(self, future.done, self.service_timeout_s):
            self.state = "FAILED"
            self.message = f"timeout waiting for fault service: {self.service_name}"
            self.finished = True
            self._publish_state()
            self.get_logger().error(self.message)
            return

        response = future.result()
        if response is None or not response.success:
            self.state = "FAILED"
            self.message = (
                "fault service returned no response"
                if response is None
                else response.message
            )
            self.finished = True
            self._publish_state()
            self.get_logger().error(self.message)
            return

        self.state = "FIRED"
        self.message = response.message
        self.finished = True
        self._publish_state()
        self.get_logger().warning(
            "V25-11D injected %s after %.2fs ROS simulation time",
            self.fault_case,
            self.trigger_delay_s,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FailureInjector()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.ready_to_fire():
                node.fire()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
