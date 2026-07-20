#!/usr/bin/env python3

import copy
import json
import math
import time
from bisect import bisect_left
from collections import deque

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
    base_yaw,
    min_relative_height,
    max_relative_height,
    footprint,
    self_filter_padding,
    resolution,
):
    if points.size == 0:
        return np.empty((0, 2), dtype=np.int64)
    relative_z = points[:, 2] - float(base_z)
    keep = np.isfinite(points).all(axis=1)
    keep &= relative_z >= float(min_relative_height)
    keep &= relative_z <= float(max_relative_height)
    if len(footprint) >= 3:
        dx = points[:, 0] - float(base_x)
        dy = points[:, 1] - float(base_y)
        cosine, sine = math.cos(base_yaw), math.sin(base_yaw)
        local_xy = np.column_stack(
            (cosine * dx + sine * dy, -sine * dx + cosine * dy)
        )
        keep &= ~inside_or_near_polygon(
            local_xy, np.asarray(footprint, dtype=np.float64), self_filter_padding
        )
    selected = points[keep, :2]
    if selected.size == 0:
        return np.empty((0, 2), dtype=np.int64)
    return np.unique(np.floor(selected / resolution).astype(np.int64), axis=0)


def inside_or_near_polygon(points, polygon, padding):
    """Return points inside a polygon or within padding of one of its edges."""
    if points.size == 0:
        return np.zeros(0, dtype=bool)
    x, y = points[:, 0], points[:, 1]
    inside = np.zeros(points.shape[0], dtype=bool)
    distance_squared = np.full(points.shape[0], np.inf)
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        crossing = (y1 > y) != (y2 > y)
        denominator = y2 - y1
        safe_denominator = denominator if abs(denominator) > 1e-12 else 1e-12
        intersection_x = (x2 - x1) * (y - y1) / safe_denominator + x1
        inside ^= crossing & (x < intersection_x)
        segment_x, segment_y = x2 - x1, y2 - y1
        length_squared = segment_x * segment_x + segment_y * segment_y
        if length_squared > 0.0:
            fraction = np.clip(
                ((x - x1) * segment_x + (y - y1) * segment_y) / length_squared,
                0.0,
                1.0,
            )
            nearest_x = x1 + fraction * segment_x
            nearest_y = y1 + fraction * segment_y
            distance_squared = np.minimum(
                distance_squared, (x - nearest_x) ** 2 + (y - nearest_y) ** 2
            )
        previous = current
    return inside | (distance_squared <= float(padding) ** 2)


def interpolate_pose(samples, timestamp, max_time_error):
    """Interpolate x/y/z/yaw at timestamp from sorted odometry samples."""
    if not samples:
        return None
    times = [sample[0] for sample in samples]
    index = bisect_left(times, timestamp)
    if index == 0:
        return samples[0][1:] if times[0] - timestamp <= max_time_error else None
    if index == len(samples):
        return samples[-1][1:] if timestamp - times[-1] <= max_time_error else None
    before, after = samples[index - 1], samples[index]
    if min(timestamp - before[0], after[0] - timestamp) > max_time_error:
        return None
    span = after[0] - before[0]
    fraction = 0.0 if span <= 0.0 else (timestamp - before[0]) / span
    yaw_delta = math.atan2(
        math.sin(after[4] - before[4]), math.cos(after[4] - before[4])
    )
    return (
        before[1] + fraction * (after[1] - before[1]),
        before[2] + fraction * (after[2] - before[2]),
        before[3] + fraction * (after[3] - before[3]),
        before[4] + fraction * yaw_delta,
    )


def rasterize_footprint_cells(
    *, base_x, base_y, base_yaw, footprint, padding, resolution
):
    """Rasterize a world-frame vehicle footprint into evidence-grid cells."""
    polygon = np.asarray(footprint, dtype=np.float64)
    if polygon.shape[0] < 3:
        return np.empty((0, 2), dtype=np.int64)
    cosine, sine = math.cos(base_yaw), math.sin(base_yaw)
    world_x = base_x + cosine * polygon[:, 0] - sine * polygon[:, 1]
    world_y = base_y + sine * polygon[:, 0] + cosine * polygon[:, 1]
    minimum_x = int(math.floor((world_x.min() - padding) / resolution))
    maximum_x = int(math.floor((world_x.max() + padding) / resolution))
    minimum_y = int(math.floor((world_y.min() - padding) / resolution))
    maximum_y = int(math.floor((world_y.max() + padding) / resolution))
    cell_x, cell_y = np.meshgrid(
        np.arange(minimum_x, maximum_x + 1, dtype=np.int64),
        np.arange(minimum_y, maximum_y + 1, dtype=np.int64),
    )
    center_x = (cell_x.reshape(-1) + 0.5) * resolution
    center_y = (cell_y.reshape(-1) + 0.5) * resolution
    dx, dy = center_x - base_x, center_y - base_y
    local = np.column_stack(
        (cosine * dx + sine * dy, -sine * dx + cosine * dy)
    )
    selected = inside_or_near_polygon(local, polygon, padding)
    return np.column_stack((cell_x.reshape(-1)[selected], cell_y.reshape(-1)[selected]))


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
        self.declare_parameter("footprint_json", "[]")
        self.declare_parameter("self_filter_padding", 0.12)
        self.declare_parameter("max_pose_time_error", 0.25)
        self.declare_parameter("pose_wait_timeout", 1.0)
        self.declare_parameter("max_pending_clouds", 100)
        self.declare_parameter("clear_swept_footprint", True)
        self.declare_parameter("sweep_clearance", 0.05)

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
            100,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._on_cloud,
            100,
        )
        # Use wall time so a final dirty map is emitted after rosbag /clock stops.
        self.create_timer(
            2.0,
            self._on_timer,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._base_map = None
        self._footprint = json.loads(str(self.get_parameter("footprint_json").value))
        self._odometry = deque(maxlen=200)
        self._pending_clouds = deque()
        self._counts = {}
        self._occupied = set()
        self._swept_free = set()
        self._dirty = False
        self._cloud_count = 0
        self._clouds_received = 0
        self._odometry_received = 0
        self._pose_mismatch_drops = 0
        self._queue_overflow_drops = 0
        self._accepted_cell_observations = 0

    def _on_odometry(self, message):
        self._odometry_received += 1
        stamp = message.header.stamp
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        position = message.pose.pose.position
        self._odometry.append(
            (
                stamp.sec + stamp.nanosec * 1e-9,
                position.x,
                position.y,
                position.z,
                yaw,
            )
        )
        if bool(self.get_parameter("clear_swept_footprint").value):
            cells = rasterize_footprint_cells(
                base_x=position.x,
                base_y=position.y,
                base_yaw=yaw,
                footprint=self._footprint,
                padding=float(self.get_parameter("sweep_clearance").value),
                resolution=float(self.get_parameter("evidence_resolution").value),
            )
            previous_size = len(self._swept_free)
            self._swept_free.update((int(x), int(y)) for x, y in cells)
            self._dirty |= len(self._swept_free) != previous_size
        self._drain_clouds()

    def _on_base_map(self, message):
        self._base_map = message
        self._dirty = True
        self._publish_if_dirty()

    def _on_timer(self):
        self._drain_clouds()
        self._publish_if_dirty()

    def _on_cloud(self, message):
        self._clouds_received += 1
        limit = int(self.get_parameter("max_pending_clouds").value)
        if len(self._pending_clouds) >= limit:
            self._pending_clouds.popleft()
            self._queue_overflow_drops += 1
        self._pending_clouds.append((message, time.monotonic()))
        self._drain_clouds()

    def _drain_clouds(self):
        if not self._odometry:
            return
        latest_time = self._odometry[-1][0]
        wait_timeout = float(self.get_parameter("pose_wait_timeout").value)
        while self._pending_clouds:
            message, queued_at = self._pending_clouds[0]
            stamp = message.header.stamp
            cloud_time = stamp.sec + stamp.nanosec * 1e-9
            if cloud_time > latest_time and time.monotonic() - queued_at < wait_timeout:
                return
            self._pending_clouds.popleft()
            pose = interpolate_pose(
                list(self._odometry),
                cloud_time,
                float(self.get_parameter("max_pose_time_error").value),
            )
            if pose is None:
                self._pose_mismatch_drops += 1
                continue
            self._process_cloud(message, pose)

    def _process_cloud(self, message, pose):
        try:
            points = point_cloud2.read_points_numpy(
                message, field_names=["x", "y", "z"], skip_nans=True
            )
        except (AssertionError, ValueError) as exc:
            self.get_logger().warning(f"unsupported PointCloud2 layout: {exc}")
            return
        if points.size == 0:
            return
        resolution = float(self.get_parameter("evidence_resolution").value)
        base_x, base_y, base_z, base_yaw = pose
        cells = select_unique_cells(
            points,
            base_x=base_x,
            base_y=base_y,
            base_z=base_z,
            base_yaw=base_yaw,
            min_relative_height=self.get_parameter("min_relative_height").value,
            max_relative_height=self.get_parameter("max_relative_height").value,
            footprint=self._footprint,
            self_filter_padding=self.get_parameter("self_filter_padding").value,
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
        if self._swept_free:
            cells = np.asarray(tuple(self._swept_free), dtype=np.int64)
            world_x = (cells[:, 0] + 0.5) * evidence_resolution
            world_y = (cells[:, 1] + 0.5) * evidence_resolution
            map_x = np.floor((world_x - origin_x) / resolution).astype(np.int64)
            map_y = np.floor((world_y - origin_y) / resolution).astype(np.int64)
            valid = (map_x >= 0) & (map_x < width) & (map_y >= 0) & (map_y < height)
            data[map_y[valid], map_x[valid]] = 0
        output.header.stamp = self.get_clock().now().to_msg()
        output.data = data.reshape(-1).astype(np.int8).tolist()
        self._publisher.publish(output)
        status = String()
        status.data = json.dumps(
            {
                "clouds": self._cloud_count,
                "clouds_received": self._clouds_received,
                "odometry_received": self._odometry_received,
                "pending_clouds": len(self._pending_clouds),
                "pose_mismatch_drops": self._pose_mismatch_drops,
                "queue_overflow_drops": self._queue_overflow_drops,
                "accepted_cell_observations": self._accepted_cell_observations,
                "pending_cells": len(self._counts),
                "occupied_cells_with_padding": len(self._occupied),
                "swept_free_cells": len(self._swept_free),
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
