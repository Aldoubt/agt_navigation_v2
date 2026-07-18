#!/usr/bin/env python3
"""Plan a validated entry-to-first-swath path without altering coverage semantics."""

from copy import deepcopy
import json
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PolygonStamped, PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from agt_coverage_planning.path_repair import repair_policy_from_profile
from agt_coverage_planning.path_validator import (
    GridMap,
    Pose2D,
    ValidatorConfig,
    footprint_shape_matches,
    validate_path,
)
from agt_ui_bridge.platform_profile import load_platform_profile
from agt_ui_bridge.semantic_io import load_semantic_map


LATCHED_QOS = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CoverageApproachPreview(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "coverage_approach_preview", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("semantic_map", "")
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter("path_topic", "/agt/coverage/path_repaired")
        self.declare_parameter("repair_report_topic", "/agt/coverage/repair_report")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("footprint_topic", "/global_costmap/published_footprint")
        self.declare_parameter("keepout_mask_topic", "/agt/map/keepout_mask")
        self.declare_parameter("semantic_status_topic", "/agt/map/semantic_status")
        self.declare_parameter("published_footprint_tolerance", 0.03)
        self.declare_parameter("maximum_sample_count", 200000)
        self.declare_parameter("retry_period", 0.50)

        semantic_path = str(self.get_parameter("semantic_map").value)
        profile_path = str(self.get_parameter("platform_profile").value)
        if not semantic_path or not profile_path:
            raise RuntimeError("semantic_map and platform_profile are required")
        semantic_map = load_semantic_map(semantic_path)
        entries = [
            feature
            for feature in semantic_map.features
            if feature.enabled and feature.feature_type == "entry_pose"
        ]
        if len(entries) != 1:
            raise RuntimeError("approach preview requires exactly one entry_pose")
        entry = entries[0]
        self.entry_pose = Pose2D(
            float(entry.coordinates[0]),
            float(entry.coordinates[1]),
            float(entry.properties["yaw"]),
        )

        profile_document = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
        self.repair_policy = repair_policy_from_profile(profile_document)
        platform = load_platform_profile(profile_path)
        self.footprint = tuple(tuple(point) for point in platform["footprint"])
        self.footprint_tolerance = float(
            self.get_parameter("published_footprint_tolerance").value
        )
        self.validator_config = ValidatorConfig(
            occupied_cost_threshold=65,
            unknown_space_policy="collision",
            outside_costmap_is_collision=True,
            maximum_sample_count=int(self.get_parameter("maximum_sample_count").value),
        )

        self.path_message = None
        self.repair_report = None
        self.costmap_message = None
        self.footprint_message = None
        self.keepout_mask_message = None
        self.semantic_status_message = None
        self.pending = False
        self.last_attempt_key = None
        self.last_report_json = ""
        self.last_path = None

        self.path_publisher = self.create_publisher(
            NavPath, "/agt/coverage/path_approach_preview", LATCHED_QOS
        )
        self.report_publisher = self.create_publisher(
            String, "/agt/coverage/approach_report", LATCHED_QOS
        )
        self.create_subscription(
            NavPath, str(self.get_parameter("path_topic").value),
            lambda msg: setattr(self, "path_message", deepcopy(msg)), LATCHED_QOS,
        )
        self.create_subscription(
            String, str(self.get_parameter("repair_report_topic").value),
            self._repair_report_callback, LATCHED_QOS,
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("costmap_topic").value),
            lambda msg: setattr(self, "costmap_message", deepcopy(msg)), LATCHED_QOS,
        )
        self.create_subscription(
            PolygonStamped, str(self.get_parameter("footprint_topic").value),
            lambda msg: setattr(self, "footprint_message", deepcopy(msg)), 10,
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("keepout_mask_topic").value),
            lambda msg: setattr(self, "keepout_mask_message", deepcopy(msg)), LATCHED_QOS,
        )
        self.create_subscription(
            DiagnosticArray, str(self.get_parameter("semantic_status_topic").value),
            lambda msg: setattr(self, "semantic_status_message", deepcopy(msg)), LATCHED_QOS,
        )
        self.planner_action = ActionClient(
            self, ComputePathToPose, str(self.get_parameter("planner_action").value)
        )
        period = float(self.get_parameter("retry_period").value)
        self.timer = self.create_timer(period, self._tick)

    def _repair_report_callback(self, message):
        try:
            self.repair_report = json.loads(message.data)
        except (json.JSONDecodeError, TypeError, ValueError):
            self.repair_report = None

    def _tick(self):
        if self.pending or not self.planner_action.server_is_ready():
            return
        required = (
            self.path_message, self.repair_report, self.costmap_message,
            self.footprint_message, self.keepout_mask_message, self.semantic_status_message,
        )
        if any(value is None for value in required):
            return
        if not self.repair_report.get("success") or not self.path_message.poses:
            return
        if not any(status.message == "LOADED" for status in self.semantic_status_message.status):
            return
        key = _stamp_ns(self.path_message)
        if key == self.last_attempt_key:
            return
        self.last_attempt_key = key
        goal_pose = _pose2d(self.path_message.poses[0])
        if math.hypot(goal_pose.x - self.entry_pose.x, goal_pose.y - self.entry_pose.y) <= 0.10:
            self._publish([], {"success": True, "state": "ALREADY_AT_START", "length": 0.0})
            return
        if not self._runtime_footprint_matches():
            self._fail("published_footprint_profile_mismatch", "runtime footprint differs from profile")
            return
        goal = ComputePathToPose.Goal()
        now = self.get_clock().now().to_msg()
        goal.start = _pose_stamped(self.entry_pose, now)
        goal.goal = _pose_stamped(goal_pose, now)
        goal.planner_id = self.repair_policy.planner_id
        goal.use_start = True
        self.pending = True
        future = self.planner_action.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("Nav2 rejected the approach goal")
            result = handle.get_result_async()
            result.add_done_callback(self._goal_result)
        except Exception as exc:
            self.pending = False
            self._fail("approach_goal_rejected", str(exc))

    def _goal_result(self, future):
        self.pending = False
        try:
            wrapped = future.result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"Nav2 action status {wrapped.status}")
            poses = [_pose2d(item) for item in wrapped.result.path.poses]
            if len(poses) < 2:
                raise RuntimeError("approach path has fewer than two poses")
            costmap = self._validate(poses, self.costmap_message, self.validator_config)
            mask = self._validate(poses, self.keepout_mask_message, self.validator_config)
            if not costmap.report.valid or not mask.report.valid:
                codes = sorted(set(costmap.report.error_codes + mask.report.error_codes))
                raise RuntimeError(",".join(codes))
            length = sum(
                math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:])
            )
            self._publish(
                poses,
                {
                    "success": True,
                    "state": "READY_PREVIEW_ONLY",
                    "planner_id": self.repair_policy.planner_id,
                    "length": length,
                    "target_path_stamp_ns": self.last_attempt_key,
                },
            )
        except Exception as exc:
            self._fail("approach_path_invalid", str(exc))

    def _runtime_footprint_matches(self):
        if self.footprint_message.header.frame_id != "map":
            return False
        runtime = tuple(
            (float(point.x), float(point.y))
            for point in self.footprint_message.polygon.points
        )
        return footprint_shape_matches(self.footprint, runtime, self.footprint_tolerance)

    def _validate(self, poses, message, config):
        return validate_path(
            poses, "map", _grid_map(message), self.footprint,
            self.repair_policy.min_turning_radius, config,
        )

    def _fail(self, code, detail):
        self._publish([], {"success": False, "state": "FAILED", "error_code": code, "detail": detail})

    def _publish(self, poses, report):
        now = self.get_clock().now().to_msg()
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = now
        path.poses = [_pose_stamped(pose, now) for pose in poses]
        self.last_path = path
        self.path_publisher.publish(path)
        message = String()
        message.data = json.dumps(report, sort_keys=True, separators=(",", ":"))
        self.last_report_json = message.data
        self.report_publisher.publish(message)


def _pose2d(stamped):
    pose = stamped.pose
    return Pose2D(float(pose.position.x), float(pose.position.y), _yaw(pose.orientation))


def _pose_stamped(pose, stamp):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    message.pose.position.x = pose.x
    message.pose.position.y = pose.y
    message.pose.orientation = Quaternion(z=math.sin(pose.yaw / 2.0), w=math.cos(pose.yaw / 2.0))
    return message


def _yaw(quaternion):
    norm = math.sqrt(sum(value * value for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)))
    if norm <= 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = (value / norm for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _grid_map(message):
    origin = message.info.origin
    return GridMap(
        width=int(message.info.width), height=int(message.info.height),
        resolution=float(message.info.resolution),
        origin_x=float(origin.position.x), origin_y=float(origin.position.y),
        origin_yaw=_yaw(origin.orientation), data=tuple(int(value) for value in message.data),
        frame_id=message.header.frame_id,
    )


def _stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageApproachPreview()
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
