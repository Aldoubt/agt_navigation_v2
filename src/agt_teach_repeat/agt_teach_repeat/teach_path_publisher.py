"""Read-only transient-local publisher for teach path previews."""

import json
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
import rclpy
from visualization_msgs.msg import Marker, MarkerArray

from .path_io import (
    TeachRepeatError,
    load_manifest,
    load_reference_path,
    manifest_reference_path,
    resolve_asset,
    verify_manifest_bindings,
)
from .ros_utils import latched_qos, path_message


class TeachPathPublisher(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("agt_teach_path_publisher", parameter_overrides=parameter_overrides)
        manifest_value = str(self.declare_parameter("manifest", "").value)
        if not manifest_value:
            raise RuntimeError("manifest parameter is required")
        self.manifest_path, self.manifest = load_manifest(manifest_value)
        self.reference = load_reference_path(
            manifest_reference_path(self.manifest_path, self.manifest),
            expected_demo_id=self.manifest["demo_id"],
        )
        self.binding = verify_manifest_bindings(self.manifest_path, self.manifest)
        self.path_publisher = self.create_publisher(
            type(path_message(self.reference, self.get_clock().now().to_msg())),
            "/agt/teach/reference_path",
            latched_qos(),
        )
        self.control_publisher = self.create_publisher(
            MarkerArray, "/agt/teach/control_points", latched_qos()
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/agt/teach/status", latched_qos()
        )
        self._publish()

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(path_message(self.reference, stamp))
        self.control_publisher.publish(self._control_markers(stamp))
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = "agt_teach_repeat/path_asset"
        status.hardware_id = self.manifest["demo_id"]
        status.level = DiagnosticStatus.OK if self.binding["valid"] else DiagnosticStatus.WARN
        status.message = (
            "ready for validation"
            if self.binding["valid"]
            else "preview only: asset binding mismatch"
        )
        status.values = [
            KeyValue(key="demo_id", value=self.manifest["demo_id"]),
            KeyValue(key="map_id", value=str(self.manifest["map"].get("map_id", ""))),
            KeyValue(key="binding_valid", value=str(self.binding["valid"]).lower()),
            KeyValue(key="binding_errors", value=",".join(self.binding["errors"])),
        ]
        diagnostics.status.append(status)
        self.status_publisher.publish(diagnostics)

    def _control_markers(self, stamp):
        path = resolve_asset(
            self.manifest_path, self.manifest["assets"].get("task_control_points", "")
        )
        document = json.loads(path.read_text(encoding="utf-8"))
        if document.get("schema_version") != 1 or not isinstance(document.get("points"), list):
            raise TeachRepeatError("schema_mismatch", "control point schema is unsupported")
        output = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        for index, point in enumerate(document["points"]):
            values = tuple(float(point[key]) for key in ("x", "y", "theta"))
            if not all(math.isfinite(value) for value in values):
                raise TeachRepeatError("non_finite_control_point", "control points must be finite")
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "teach_control_points"
            marker.id = index + 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = values[0]
            marker.pose.position.y = values[1]
            marker.pose.orientation.z = math.sin(values[2] * 0.5)
            marker.pose.orientation.w = math.cos(values[2] * 0.5)
            marker.scale.x = 0.35
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.r = 0.10
            marker.color.g = 0.55
            marker.color.b = 0.95
            marker.color.a = 0.95
            output.markers.append(marker)
        return output


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeachPathPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if "node" in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
