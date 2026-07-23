#!/usr/bin/env python3

"""Bound the expensive full-map OctoMap projection input rate."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class OctomapCloudThrottle(Node):
    def __init__(self) -> None:
        super().__init__("agt_map_processing_octomap_cloud_throttle")
        input_topic = str(self.declare_parameter("input_topic", "/agt/mapping/registered_points_lidar").value)
        output_topic = str(self.declare_parameter("output_topic", "/agt/mapping/octomap_points").value)
        rate_hz = float(self.declare_parameter("max_rate_hz", 0.2).value)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("max_rate_hz must be finite and positive")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._period_sec = 1.0 / rate_hz
        self._last_publish = float("-inf")
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self._subscription = self.create_subscription(PointCloud2, input_topic, self._callback, qos)

    def _callback(self, message: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_publish < self._period_sec:
            return
        self._last_publish = now
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OctomapCloudThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
