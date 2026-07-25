#!/usr/bin/env python3

"""Bound the expensive full-map OctoMap projection input rate."""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class OctomapCloudThrottle(Node):
    def __init__(self) -> None:
        super().__init__("agt_map_processing_octomap_cloud_throttle")
        input_topic = str(self.declare_parameter("input_topic", "/agt/mapping/registered_points_lidar").value)
        output_topic = str(self.declare_parameter("output_topic", "/agt/mapping/octomap_points").value)
        rate_hz = float(self.declare_parameter("max_rate_hz", 0.2).value)
        voxel_leaf_size = float(self.declare_parameter("voxel_leaf_size", 0.10).value)
        max_points = int(self.declare_parameter("max_points", 8000).value)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("max_rate_hz must be finite and positive")
        if not math.isfinite(voxel_leaf_size) or voxel_leaf_size < 0.0:
            raise ValueError("voxel_leaf_size must be finite and non-negative")
        if max_points < 0:
            raise ValueError("max_points must be non-negative")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._period_sec = 1.0 / rate_hz
        self._voxel_leaf_size = voxel_leaf_size
        self._max_points = max_points
        self._last_publish = float("-inf")
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self._subscription = self.create_subscription(PointCloud2, input_topic, self._callback, qos)

    def _callback(self, message: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_publish < self._period_sec:
            return
        self._last_publish = now
        cloud = self._downsample(message)
        if cloud is not None:
            self._publisher.publish(cloud)

    def _downsample(self, message: PointCloud2):
        points = point_cloud2.read_points_numpy(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        if points.size == 0:
            return None
        points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
        points = points[np.isfinite(points).all(axis=1)]
        if points.size == 0:
            return None

        if self._voxel_leaf_size > 0.0:
            keys = np.floor(points / self._voxel_leaf_size).astype(np.int64)
            _, first_indices = np.unique(keys, axis=0, return_index=True)
            points = points[np.sort(first_indices)]

        if self._max_points > 0 and points.shape[0] > self._max_points:
            indices = np.linspace(
                0, points.shape[0] - 1, self._max_points, dtype=np.int64
            )
            points = points[indices]

        return point_cloud2.create_cloud_xyz32(message.header, points.tolist())


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
