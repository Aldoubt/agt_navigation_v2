#!/usr/bin/env python3

"""Automated V25-11A Gazebo simulation baseline acceptance smoke."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener


class BaselineObserver(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11a_baseline_acceptance")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.report_path = Path(
            str(
                self.declare_parameter(
                    "report_path", "/tmp/agt_v25_11a_baseline_result.json"
                ).value
            )
        )
        self.counts = {"clock": 0, "odom": 0, "scan": 0, "imu": 0}
        self.first_wall = {}
        self.last_wall = {}
        self.initial_xy = None
        self.latest_xy = None
        self.cmd_pub = self.create_publisher(Twist, "/agt/safety/cmd_vel", 10)
        self.create_subscription(Clock, "/clock", lambda msg: self._seen("clock"), 10)
        self.create_subscription(
            Odometry, "/agt/mapping/odometry", self._odom, 20
        )
        self.create_subscription(
            LaserScan,
            "/agt/sensors/lidar/scan",
            lambda msg: self._seen("scan"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/agt/sensors/imu/data",
            lambda msg: self._seen("imu"),
            qos_profile_sensor_data,
        )
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def _seen(self, name: str) -> None:
        now = time.monotonic()
        self.counts[name] += 1
        self.first_wall.setdefault(name, now)
        self.last_wall[name] = now

    def _odom(self, message: Odometry) -> None:
        self._seen("odom")
        xy = (float(message.pose.pose.position.x), float(message.pose.pose.position.y))
        if self.initial_xy is None:
            self.initial_xy = xy
        self.latest_xy = xy

    def rate(self, name: str) -> float:
        elapsed = self.last_wall.get(name, 0.0) - self.first_wall.get(name, 0.0)
        return self.counts[name] / elapsed if elapsed > 0.0 else 0.0


def _spin_until(node: BaselineObserver, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return bool(predicate())


def _publish_motion(node: BaselineObserver, duration_s: float = 2.0) -> None:
    command = Twist()
    command.linear.x = 0.20
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.cmd_pub.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.cmd_pub.publish(Twist())
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaselineObserver()
    result = {"gate": "V25-11A", "status": "FAIL", "checks": {}}
    exit_code = 1
    try:
        warmed = _spin_until(
            node,
            lambda: node.counts["odom"] >= 20
            and node.counts["scan"] >= 10
            and node.counts["imu"] >= 50
            and node.counts["clock"] >= 20,
            15.0,
        )
        result["checks"]["topics_warm"] = warmed
        result["rates_hz"] = {
            name: round(node.rate(name), 3) for name in ("odom", "scan", "imu")
        }

        tf_checks = {}
        for target, source, key in (
            ("odom", "base_footprint", "odom_base"),
            ("base_footprint", "lidar_link", "base_lidar"),
            ("base_footprint", "imu_link", "base_imu"),
        ):
            tf_checks[key] = node.tf.can_transform(target, source, Time())
        tf_checks["map_odom_absent"] = not node.tf.can_transform("map", "odom", Time())
        result["checks"]["tf"] = tf_checks

        before = node.latest_xy
        _publish_motion(node)
        after = node.latest_xy
        displacement = 0.0
        if before is not None and after is not None:
            displacement = math.hypot(after[0] - before[0], after[1] - before[1])
        result["measured_displacement_m"] = displacement
        result["checks"]["command_path_moves_vehicle"] = displacement >= 0.05

        rate_checks = {
            "odom_ge_5_hz": node.rate("odom") >= 5.0,
            "scan_ge_5_hz": node.rate("scan") >= 5.0,
            "imu_ge_50_hz": node.rate("imu") >= 50.0,
        }
        result["checks"]["rates"] = rate_checks

        passed = (
            warmed
            and all(tf_checks.values())
            and all(rate_checks.values())
            and result["checks"]["command_path_moves_vehicle"]
        )
        result["status"] = "PASS" if passed else "FAIL"
        exit_code = 0 if passed else 1
    finally:
        node.cmd_pub.publish(Twist())
        node.report_path.parent.mkdir(parents=True, exist_ok=True)
        node.report_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
