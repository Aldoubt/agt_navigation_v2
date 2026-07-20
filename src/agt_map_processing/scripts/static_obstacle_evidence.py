#!/usr/bin/env python3

import copy
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def select_unique_cells(
    points,
    *,
    base_x,
    base_y,
    base_z,
    min_relative_height,
    max_relative_height,
    self_filter_radius,
    resolution,
):
    if points.size == 0:
        return np.empty((0, 2), dtype=np.int64)
    relative_z = points[:, 2] - float(base_z)
    keep = np.isfinite(points).all(axis=1)
    keep &= relative_z >= float(min_relative_height)
    keep &= relative_z <= float(max_relative_height)
    if self_filter_radius > 0.0:
        dx = points[:, 0] - float(base_x)
        dy = points[:, 1] - float(base_y)
        keep &= (dx * dx + dy * dy) >= self_filter_radius * self_filter_radius
    selected = points[keep, :2]
    if selected.size == 0:
        return np.empty((0, 2), dtype=np.int64)
    return np.unique(np.floor(selected / resolution).astype(np.int64), axis=0)


class StaticObstacleEvidence(Node):
    """Overlay repeatable, base-relative obstacle hits onto a ray-traced map."""

    def __init__(self):
        super().__init__("agt_static_obstacle_evidence")
        self.declare_parameter("base_map_topic", "/agt/map/octomap_occupancy")
        self.declare_parameter("cloud_topic", "/agt/mapping/registered_points")
        self.declare_parameter("odometry_topic", "/agt/mapping/odometry")
        self.declare_parameter("output_topic", "/agt/map/mapping_occupancy")
        self.declare_parameter("min_relative_height", 0.05)
        self.declare_parameter("max_relative_height", 2.00)
        self.declare_parameter("evidence_resolution", 0.05)
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("obstacle_padding", 0.05)
        self.declare_parameter("self_filter_radius", 0.0)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("output_topic").value), qos
        )
        self._status = self.create_publisher(
            String, "/agt/map/static_obstacle_evidence_status", qos
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("base_map_topic").value),
            self._on_base_map,
            qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._on_odometry,
            20,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._on_cloud,
            10,
        )
        # Use wall time so a final dirty map is emitted after rosbag /clock stops.
        self.create_timer(
            2.0,
            self._publish_if_dirty,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._base_map = None
        self._base_pose = None
        self._counts = {}
        self._occupied = set()
        self._dirty = False
        self._cloud_count = 0
        self._accepted_cell_observations = 0

    def _on_odometry(self, message):
        self._base_pose = message.pose.pose.position

    def _on_base_map(self, message):
        self._base_map = message
        self._dirty = True
        self._publish_if_dirty()

    def _on_cloud(self, message):
        if self._base_pose is None:
            return
        try:
            points = point_cloud2.read_points_numpy(
                message, field_names=["x", "y", "z"], skip_nans=True
            )
        except (AssertionError, ValueError) as exc:
            self.get_logger().warning(f"unsupported PointCloud2 layout: {exc}")
            return
        if points.size == 0:
            return
        radius = float(self.get_parameter("self_filter_radius").value)
        resolution = float(self.get_parameter("evidence_resolution").value)
        cells = select_unique_cells(
            points,
            base_x=self._base_pose.x,
            base_y=self._base_pose.y,
            base_z=self._base_pose.z,
            min_relative_height=self.get_parameter("min_relative_height").value,
            max_relative_height=self.get_parameter("max_relative_height").value,
            self_filter_radius=radius,
            resolution=resolution,
        )
        self._cloud_count += 1
        self._accepted_cell_observations += int(cells.shape[0])
        if cells.size == 0:
            return
        threshold = int(self.get_parameter("min_observations").value)
        padding_cells = int(
            math.ceil(float(self.get_parameter("obstacle_padding").value) / resolution)
        )
        changed = False
        for ix, iy in cells:
            key = (int(ix), int(iy))
            if key in self._occupied:
                continue
            count = self._counts.get(key, 0) + 1
            if count < threshold:
                self._counts[key] = count
                continue
            self._counts.pop(key, None)
            for dx in range(-padding_cells, padding_cells + 1):
                for dy in range(-padding_cells, padding_cells + 1):
                    self._occupied.add((key[0] + dx, key[1] + dy))
            changed = True
        self._dirty |= changed

    def _publish_if_dirty(self):
        if not self._dirty or self._base_map is None:
            return
        output = copy.deepcopy(self._base_map)
        width = int(output.info.width)
        height = int(output.info.height)
        resolution = float(output.info.resolution)
        origin_x = float(output.info.origin.position.x)
        origin_y = float(output.info.origin.position.y)
        evidence_resolution = float(self.get_parameter("evidence_resolution").value)
        data = np.asarray(output.data, dtype=np.int16).reshape((height, width)).copy()
        if self._occupied:
            cells = np.asarray(tuple(self._occupied), dtype=np.int64)
            world_x = (cells[:, 0] + 0.5) * evidence_resolution
            world_y = (cells[:, 1] + 0.5) * evidence_resolution
            map_x = np.floor((world_x - origin_x) / resolution).astype(np.int64)
            map_y = np.floor((world_y - origin_y) / resolution).astype(np.int64)
            valid = (map_x >= 0) & (map_x < width) & (map_y >= 0) & (map_y < height)
            data[map_y[valid], map_x[valid]] = 100
        output.header.stamp = self.get_clock().now().to_msg()
        output.data = data.reshape(-1).astype(np.int8).tolist()
        self._publisher.publish(output)
        status = String()
        status.data = json.dumps(
            {
                "clouds": self._cloud_count,
                "accepted_cell_observations": self._accepted_cell_observations,
                "pending_cells": len(self._counts),
                "occupied_cells_with_padding": len(self._occupied),
                "output_width": width,
                "output_height": height,
                "resolution": resolution,
            },
            sort_keys=True,
        )
        self._status.publish(status)
        self._dirty = False


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleEvidence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # A second SIGINT may arrive while launch is tearing subscriptions down.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
