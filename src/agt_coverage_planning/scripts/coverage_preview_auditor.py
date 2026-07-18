#!/usr/bin/env python3
"""Publish non-actionable collision diagnostics for an offline preview path."""

import json
import math

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from agt_coverage_planning.path_validator import (
    GridMap,
    PathValidationError,
    Pose2D,
    ValidatorConfig,
)
from agt_coverage_planning.preview_audit import audit_preview_path
from agt_ui_bridge.platform_profile import load_platform_profile


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CoveragePreviewAuditor(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "coverage_preview_auditor", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("path_topic", "/agt/coverage/path_preview")
        self.declare_parameter("base_map_topic", "/agt/map/global_occupancy")
        self.declare_parameter("keepout_mask_topic", "/agt/map/keepout_mask")
        self.declare_parameter("report_topic", "/agt/coverage/preview_audit")
        self.declare_parameter(
            "collision_topic", "/agt/coverage/preview_collision_poses"
        )
        self.declare_parameter("occupied_cost_threshold", 65)
        self.declare_parameter("unknown_space_policy", "collision")
        self.declare_parameter("outside_costmap_is_collision", True)
        self.declare_parameter("maximum_sample_count", 200000)
        self.declare_parameter("maximum_visualized_collisions", 500)

        profile_path = str(self.get_parameter("platform_profile").value)
        if not profile_path:
            raise RuntimeError("platform_profile parameter is required")
        platform = load_platform_profile(profile_path)
        self.footprint = tuple(tuple(point) for point in platform["footprint"])
        self.min_turning_radius = float(platform["min_turning_radius"])
        self.config = ValidatorConfig(
            occupied_cost_threshold=int(
                self.get_parameter("occupied_cost_threshold").value
            ),
            unknown_space_policy=str(
                self.get_parameter("unknown_space_policy").value
            ),
            outside_costmap_is_collision=bool(
                self.get_parameter("outside_costmap_is_collision").value
            ),
            maximum_sample_count=int(
                self.get_parameter("maximum_sample_count").value
            ),
        )
        self.maximum_visualized_collisions = int(
            self.get_parameter("maximum_visualized_collisions").value
        )
        if self.maximum_visualized_collisions < 0:
            raise RuntimeError("maximum_visualized_collisions must be non-negative")

        self.path = None
        self.base_map = None
        self.keepout_mask = None
        self.last_report = None
        self.last_collisions = None
        self.report_publisher = self.create_publisher(
            String, str(self.get_parameter("report_topic").value), LATCHED_QOS
        )
        self.collision_publisher = self.create_publisher(
            PoseArray, str(self.get_parameter("collision_topic").value), LATCHED_QOS
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self._path_callback,
            LATCHED_QOS,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("base_map_topic").value),
            self._base_map_callback,
            LATCHED_QOS,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("keepout_mask_topic").value),
            self._keepout_callback,
            LATCHED_QOS,
        )

    def _path_callback(self, message):
        self.path = message
        self._audit()

    def _base_map_callback(self, message):
        self.base_map = message
        self._audit()

    def _keepout_callback(self, message):
        self.keepout_mask = message
        self._audit()

    def _audit(self):
        if self.path is None or self.base_map is None or self.keepout_mask is None:
            return
        if not self.path.poses:
            self._publish({}, [])
            return
        try:
            poses = _path_poses(self.path)
            result = audit_preview_path(
                poses,
                _grid(self.base_map),
                _grid(self.keepout_mask),
                self.footprint,
                self.min_turning_radius,
                self.config,
            )
            report = dict(result.report)
            report["path_stamp_ns"] = _stamp_ns(self.path.header.stamp)
            collision_samples = list(result.collision_samples)
        except (KeyError, OSError, PathValidationError, TypeError, ValueError) as exc:
            report = {
                "schema_version": "1.0",
                "status": "FAILED",
                "advisory_only": True,
                "eligible_for_execution": False,
                "error_code": getattr(exc, "code", "preview_audit_failed"),
                "detail": str(exc),
                "source_topic": "/agt/coverage/path_preview",
                "path_stamp_ns": _stamp_ns(self.path.header.stamp),
            }
            collision_samples = []
        self._publish(report, collision_samples)

    def _publish(self, report, collision_samples):
        report_message = String()
        report_message.data = json.dumps(
            report, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        self.last_report = report
        self.report_publisher.publish(report_message)

        collisions = PoseArray()
        collisions.header.frame_id = "map"
        collisions.header.stamp = (
            self.path.header.stamp
            if self.path is not None
            else self.get_clock().now().to_msg()
        )
        selected = _evenly_spaced(
            collision_samples, self.maximum_visualized_collisions
        )
        collisions.poses = [_pose(sample.pose) for sample in selected]
        self.last_collisions = collisions
        self.collision_publisher.publish(collisions)


def _grid(message):
    origin = message.info.origin
    return GridMap(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution=float(message.info.resolution),
        origin_x=float(origin.position.x),
        origin_y=float(origin.position.y),
        origin_yaw=_quaternion_yaw(origin.orientation),
        data=tuple(int(value) for value in message.data),
        frame_id=message.header.frame_id,
    )


def _path_poses(message):
    if message.header.frame_id != "map":
        raise PathValidationError(
            "invalid_path_frame", "preview path must use map frame"
        )
    return [
        Pose2D(
            float(stamped.pose.position.x),
            float(stamped.pose.position.y),
            _quaternion_yaw(stamped.pose.orientation),
        )
        for stamped in message.poses
    ]


def _quaternion_yaw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise PathValidationError(
            "invalid_orientation", "orientation must contain finite values"
        )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise PathValidationError(
            "invalid_orientation", "orientation quaternion norm is zero"
        )
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose(pose):
    output = Pose()
    output.position.x = pose.x
    output.position.y = pose.y
    output.orientation = Quaternion(
        z=math.sin(pose.yaw * 0.5), w=math.cos(pose.yaw * 0.5)
    )
    return output


def _evenly_spaced(values, maximum_count):
    if maximum_count <= 0 or not values:
        return []
    if maximum_count == 1:
        return [values[0]]
    if len(values) <= maximum_count:
        return values
    return [
        values[round(index * (len(values) - 1) / (maximum_count - 1))]
        for index in range(maximum_count)
    ]


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePreviewAuditor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
