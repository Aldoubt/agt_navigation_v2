#!/usr/bin/env python3

"""Bound the expensive full-map OctoMap projection input rate."""

import math
import threading

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LatestCloudSlot:
    """Keep at most one unprocessed cloud without changing its header."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest = None
        self._received = 0
        self._superseded = 0

    def update(self, message) -> None:
        with self._lock:
            self._received += 1
            if self._latest is not None:
                self._superseded += 1
            self._latest = message

    def take(self):
        with self._lock:
            message = self._latest
            self._latest = None
            return message

    @property
    def received(self) -> int:
        with self._lock:
            return self._received

    @property
    def superseded(self) -> int:
        with self._lock:
            return self._superseded


class ProjectionGate:
    """Allow one cloud in flight until OctoMap publishes its resulting grid."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._in_flight_since = None
        self._timeouts = 0

    def ready(self, now_sec: float, timeout_sec: float) -> bool:
        with self._lock:
            if self._in_flight_since is None:
                return True
            if now_sec - self._in_flight_since < timeout_sec:
                return False
            self._in_flight_since = None
            self._timeouts += 1
            return True

    def mark_published(self, now_sec: float) -> None:
        with self._lock:
            self._in_flight_since = now_sec

    def acknowledge(self) -> None:
        with self._lock:
            self._in_flight_since = None

    @property
    def timeouts(self) -> int:
        with self._lock:
            return self._timeouts


class OctomapCloudThrottle(Node):
    def __init__(self) -> None:
        super().__init__("agt_map_processing_octomap_cloud_throttle")
        input_topic = str(self.declare_parameter("input_topic", "/agt/mapping/registered_points_lidar").value)
        output_topic = str(self.declare_parameter("output_topic", "/agt/mapping/octomap_points").value)
        projected_map_input_topic = str(
            self.declare_parameter(
                "projected_map_input_topic",
                "/agt/map/mapping_occupancy_raw",
            ).value
        )
        map_output_topic = str(
            self.declare_parameter(
                "map_output_topic", "/agt/map/mapping_occupancy"
            ).value
        )
        rate_hz = float(self.declare_parameter("max_rate_hz", 0.2).value)
        voxel_leaf_size = float(self.declare_parameter("voxel_leaf_size", 0.10).value)
        max_points = int(self.declare_parameter("max_points", 8000).value)
        processing_timeout_sec = float(
            self.declare_parameter("processing_timeout_sec", 60.0).value
        )
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("max_rate_hz must be finite and positive")
        if not math.isfinite(voxel_leaf_size) or voxel_leaf_size < 0.0:
            raise ValueError("voxel_leaf_size must be finite and non-negative")
        if max_points < 0:
            raise ValueError("max_points must be non-negative")
        if not math.isfinite(processing_timeout_sec) or processing_timeout_sec <= 0.0:
            raise ValueError("processing_timeout_sec must be finite and positive")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        feedback_qos = QoSProfile(depth=1)
        feedback_qos.reliability = ReliabilityPolicy.RELIABLE
        feedback_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        raw_map_qos = QoSProfile(depth=5)
        raw_map_qos.reliability = ReliabilityPolicy.RELIABLE
        raw_map_qos.durability = DurabilityPolicy.VOLATILE
        self._period_sec = 1.0 / rate_hz
        self._voxel_leaf_size = voxel_leaf_size
        self._max_points = max_points
        self._processing_timeout_sec = processing_timeout_sec
        self._pending = LatestCloudSlot()
        self._projection_gate = ProjectionGate()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self._map_publisher = self.create_publisher(
            OccupancyGrid, map_output_topic, feedback_qos
        )
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._store_latest, qos
        )
        self._feedback_subscription = self.create_subscription(
            OccupancyGrid,
            projected_map_input_topic,
            self._projection_complete,
            raw_map_qos,
        )
        self._timer = self.create_timer(
            self._period_sec,
            self._publish_latest,
            clock=self._steady_clock,
        )

    def _store_latest(self, message: PointCloud2) -> None:
        self._pending.update(message)

    def _projection_complete(self, message: OccupancyGrid) -> None:
        self._projection_gate.acknowledge()
        self._map_publisher.publish(message)

    def _publish_latest(self) -> None:
        now_sec = self._steady_clock.now().nanoseconds / 1_000_000_000.0
        previous_timeouts = self._projection_gate.timeouts
        if not self._projection_gate.ready(now_sec, self._processing_timeout_sec):
            return
        if self._projection_gate.timeouts != previous_timeouts:
            self.get_logger().warning(
                "OctoMap projection acknowledgement timed out after %.1f s; "
                "releasing only the newest pending cloud",
                self._processing_timeout_sec,
            )
        message = self._pending.take()
        if message is None:
            return
        cloud = self._downsample(message)
        if cloud is not None:
            self._publisher.publish(cloud)
            self._projection_gate.mark_published(now_sec)

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
