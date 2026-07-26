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
from .route_annotations import (
    load_route_annotations,
    route_annotation_document,
)


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
        annotation_value = self.manifest["assets"].get("route_annotations")
        if annotation_value:
            self.annotations = load_route_annotations(
                resolve_asset(self.manifest_path, annotation_value),
                expected_demo_id=self.manifest["demo_id"],
                expected_reference_path_sha256=self.manifest["assets"][
                    "reference_path_sha256"
                ],
            )
        else:
            self.annotations = route_annotation_document(
                self.manifest["demo_id"],
                self.reference,
                self.manifest["assets"]["reference_path_sha256"],
            )
        self.path_publisher = self.create_publisher(
            type(path_message(self.reference, self.get_clock().now().to_msg())),
            "/agt/teach/reference_path",
            latched_qos(),
        )
        self.control_publisher = self.create_publisher(
            MarkerArray, "/agt/teach/control_points", latched_qos()
        )
        self.annotation_publisher = self.create_publisher(
            MarkerArray, "/agt/teach/route_annotations", latched_qos()
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/agt/teach/status", latched_qos()
        )
        self._publish()

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(path_message(self.reference, stamp))
        self.control_publisher.publish(self._control_markers(stamp))
        self.annotation_publisher.publish(self._annotation_markers(stamp))
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

    @staticmethod
    def _marker_color(marker, event_type):
        colors = {
            "START": (0.00, 0.55, 0.48),
            "END": (0.84, 0.18, 0.18),
            "TURN_LEFT": (0.16, 0.62, 0.35),
            "TURN_RIGHT": (0.96, 0.55, 0.10),
            "U_TURN_LEFT": (0.58, 0.25, 0.78),
            "U_TURN_RIGHT": (0.58, 0.25, 0.78),
            "IN_PLACE_LEFT": (0.86, 0.22, 0.36),
            "IN_PLACE_RIGHT": (0.86, 0.22, 0.36),
        }
        red, green, blue = colors[event_type]
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.95

    def _annotation_markers(self, stamp):
        output = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        output.markers.append(clear)

        for index, direction in enumerate(self.annotations["directions"]):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "teach_route_direction"
            marker.id = index + 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = float(direction["x"])
            marker.pose.position.y = float(direction["y"])
            marker.pose.orientation.z = math.sin(float(direction["yaw"]) * 0.5)
            marker.pose.orientation.w = math.cos(float(direction["yaw"]) * 0.5)
            marker.scale.x = 0.45
            marker.scale.y = 0.10
            marker.scale.z = 0.10
            marker.color.r = 0.10
            marker.color.g = 0.42
            marker.color.b = 0.90
            marker.color.a = 0.82
            output.markers.append(marker)

        for index, event in enumerate(self.annotations["events"]):
            event_type = str(event["type"])
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "teach_route_event"
            marker.id = index + 1
            marker.type = (
                Marker.SPHERE if event_type in {"START", "END"} else Marker.ARROW
            )
            marker.action = Marker.ADD
            marker.text = event_type
            marker.pose.position.x = float(event["x"])
            marker.pose.position.y = float(event["y"])
            marker.pose.orientation.z = math.sin(float(event["yaw"]) * 0.5)
            marker.pose.orientation.w = math.cos(float(event["yaw"]) * 0.5)
            marker.scale.x = 0.65
            marker.scale.y = 0.18
            marker.scale.z = 0.18
            self._marker_color(marker, event_type)
            output.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = "teach_route_event_label"
            label.id = index + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.text = event_type
            label.pose.position.x = float(event["x"])
            label.pose.position.y = float(event["y"])
            label.pose.position.z = 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.30
            self._marker_color(label, event_type)
            output.markers.append(label)
        return output

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
