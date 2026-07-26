"""Advisory swept-footprint corridor audit; it has no map write interface."""

from copy import deepcopy
import json

from agt_coverage_planning.path_validator import ValidatorConfig
from agt_ui_bridge.platform_profile import load_platform_profile
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from .corridor_audit import audit_corridor
from .path_io import atomic_write_json, load_manifest
from .ros_utils import (
    footprint_markers,
    grid_map,
    latched_qos,
    message_poses,
    pose_message,
)


class CorridorAuditor(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("agt_teach_corridor_auditor", parameter_overrides=parameter_overrides)
        manifest_value = str(self.declare_parameter("manifest", "").value)
        self.declare_parameter("path_topic", "/agt/teach/reference_path")
        self.declare_parameter("costmap_topic", "/agt/map/global_occupancy")
        self.declare_parameter("platform_profile", "")
        self.declare_parameter("occupied_cost_threshold", 65)
        self.declare_parameter("unknown_space_policy", "collision")
        self.declare_parameter("outside_costmap_is_collision", True)
        if not manifest_value:
            raise RuntimeError("manifest parameter is required")
        self.manifest_path, self.manifest = load_manifest(manifest_value)
        profile = str(self.get_parameter("platform_profile").value) or self.manifest[
            "platform"
        ]["profile"]
        platform = load_platform_profile(profile)
        self.footprint = tuple(tuple(point) for point in platform["footprint"])
        self.min_turning_radius = float(platform["min_turning_radius"])
        self.config = ValidatorConfig(
            occupied_cost_threshold=int(self.get_parameter("occupied_cost_threshold").value),
            unknown_space_policy=str(self.get_parameter("unknown_space_policy").value),
            outside_costmap_is_collision=bool(
                self.get_parameter("outside_costmap_is_collision").value
            ),
        )
        self.path = None
        self.costmap = None
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/agt/teach/corridor_markers", latched_qos()
        )
        self.conflict_publisher = self.create_publisher(
            PoseArray, "/agt/teach/corridor_conflicts", latched_qos()
        )
        self.report_publisher = self.create_publisher(
            String, "/agt/teach/corridor_report", latched_qos()
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
        self._audit()

    def _costmap_callback(self, message):
        self.costmap = deepcopy(message)
        self._audit()

    def _audit(self):
        if self.path is None or self.costmap is None:
            return
        poses = message_poses(self.path)
        result, report = audit_corridor(
            poses,
            grid_map(self.costmap),
            self.footprint,
            self.min_turning_radius,
            self.config,
            self.manifest["demo_id"],
        )
        atomic_write_json(self.manifest_path.parent / "audit" / "corridor_conflicts.json", report)
        stamp = self.get_clock().now().to_msg()
        markers = footprint_markers(
            result.samples[:: max(1, len(result.samples) // 300)],
            self.footprint,
            stamp,
            "teach_swept_corridor",
            (0.10, 0.72, 0.42, 0.32),
            maximum_count=300,
        )
        atomic_write_json(
            self.manifest_path.parent / "audit" / "corridor_markers.json",
            {
                "schema_version": 1,
                "demo_id": self.manifest["demo_id"],
                "frame_id": "map",
                "sampled_poses": [
                    {
                        "x": sample.pose.x,
                        "y": sample.pose.y,
                        "yaw": sample.pose.yaw,
                    }
                    for sample in result.samples[:: max(1, len(result.samples) // 300)]
                ],
                "eligible_for_automatic_map_edit": False,
            },
        )
        self.marker_publisher.publish(markers)
        conflicts = PoseArray()
        conflicts.header.frame_id = "map"
        conflicts.header.stamp = stamp
        conflicts.poses = [pose_message(sample.pose) for sample in result.collision_samples]
        self.conflict_publisher.publish(conflicts)
        message = String()
        message.data = json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False)
        self.report_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorAuditor()
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
