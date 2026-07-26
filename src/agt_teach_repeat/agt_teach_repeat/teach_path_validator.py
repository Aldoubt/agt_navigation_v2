"""Validate a teach path with the canonical platform footprint and map grid."""

from copy import deepcopy
import json

from agt_coverage_planning.path_validator import (
    PathValidationError,
    ValidatorConfig,
    validate_path,
)
from agt_ui_bridge.platform_profile import load_platform_profile
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from .path_io import atomic_write_json, load_manifest, verify_manifest_bindings
from .ros_utils import (
    footprint_markers,
    grid_map,
    latched_qos,
    message_poses,
    pose_message,
)


class TeachPathValidator(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("agt_teach_path_validator", parameter_overrides=parameter_overrides)
        manifest_value = str(self.declare_parameter("manifest", "").value)
        self.declare_parameter("path_topic", "/agt/teach/reference_path")
        self.declare_parameter("costmap_topic", "/agt/map/global_occupancy")
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("occupied_cost_threshold", 65)
        self.declare_parameter("unknown_space_policy", "collision")
        self.declare_parameter("outside_costmap_is_collision", True)
        self.declare_parameter("maximum_sample_count", 200000)
        if not manifest_value:
            raise RuntimeError("manifest parameter is required")
        self.manifest_path, self.manifest = load_manifest(manifest_value)
        profile_value = str(self.get_parameter("platform_profile").value) or str(
            self.manifest["platform"]["profile"]
        )
        platform = load_platform_profile(profile_value)
        self.footprint = tuple(tuple(point) for point in platform["footprint"])
        self.min_turning_radius = float(platform["min_turning_radius"])
        self.config = ValidatorConfig(
            occupied_cost_threshold=int(self.get_parameter("occupied_cost_threshold").value),
            unknown_space_policy=str(self.get_parameter("unknown_space_policy").value),
            outside_costmap_is_collision=bool(
                self.get_parameter("outside_costmap_is_collision").value
            ),
            maximum_sample_count=int(self.get_parameter("maximum_sample_count").value),
        )
        self.path = None
        self.costmap = None
        self.last_report = None
        self.last_validated_path = None
        self.validated_publisher = self.create_publisher(
            Path, "/agt/teach/path_validated", latched_qos()
        )
        self.collision_publisher = self.create_publisher(
            PoseArray, "/agt/teach/collision_poses", latched_qos()
        )
        self.footprint_publisher = self.create_publisher(
            MarkerArray, "/agt/teach/footprint_markers", latched_qos()
        )
        self.report_publisher = self.create_publisher(
            String, "/agt/teach/validation_report", latched_qos()
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self._path_callback,
            latched_qos(),
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._costmap_callback,
            latched_qos(),
        )

    def _path_callback(self, message):
        self.path = deepcopy(message)
        self._validate()

    def _costmap_callback(self, message):
        self.costmap = deepcopy(message)
        self._validate()

    def _validate(self):
        if self.path is None or self.costmap is None:
            return
        try:
            poses = message_poses(self.path)
            result = validate_path(
                poses,
                self.path.header.frame_id,
                grid_map(self.costmap),
                self.footprint,
                self.min_turning_radius,
                self.config,
            )
            report = result.report.to_dict()
            report["demo_id"] = self.manifest["demo_id"]
            binding = verify_manifest_bindings(self.manifest_path, self.manifest)
            report["asset_binding_valid"] = binding["valid"]
            report["asset_binding_errors"] = binding["errors"]
            report["eligible_for_execution"] = bool(result.report.valid and binding["valid"])
            self._publish(result, report)
        except (PathValidationError, TypeError, ValueError) as exc:
            code = getattr(exc, "code", "validator_input_error")
            report = {
                "valid": False,
                "demo_id": self.manifest["demo_id"],
                "eligible_for_execution": False,
                "error_codes": [str(code)],
            }
            self._publish_failure(report)

    def _publish(self, result, report):
        stamp = self.get_clock().now().to_msg()
        validated = Path()
        validated.header.frame_id = "map"
        validated.header.stamp = stamp
        if result.report.valid:
            validated = deepcopy(self.path)
            validated.header.stamp = stamp
        self.last_validated_path = validated
        self.validated_publisher.publish(validated)
        collision = PoseArray()
        collision.header.frame_id = "map"
        collision.header.stamp = stamp
        collision.poses = [pose_message(sample.pose) for sample in result.collision_samples]
        self.collision_publisher.publish(collision)
        self.footprint_publisher.publish(
            footprint_markers(
                result.invalid_samples,
                self.footprint,
                stamp,
                "invalid_teach_footprint",
                (0.92, 0.18, 0.12, 0.85),
            )
        )
        self._publish_report(report)

    def _publish_failure(self, report):
        stamp = self.get_clock().now().to_msg()
        empty = Path()
        empty.header.frame_id = "map"
        empty.header.stamp = stamp
        self.last_validated_path = empty
        self.validated_publisher.publish(empty)
        collision = PoseArray()
        collision.header.frame_id = "map"
        collision.header.stamp = stamp
        self.collision_publisher.publish(collision)
        self.footprint_publisher.publish(
            footprint_markers(
                [],
                self.footprint,
                stamp,
                "invalid_teach_footprint",
                (0.92, 0.18, 0.12, 0.85),
            )
        )
        self._publish_report(report)

    def _publish_report(self, report):
        self.last_report = report
        atomic_write_json(self.manifest_path.parent / "audit" / "path_validation.json", report)
        message = String()
        message.data = json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False)
        self.report_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TeachPathValidator()
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
