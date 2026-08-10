#!/usr/bin/env python3

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from agt_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def spin_until(node: Node, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return bool(predicate())


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class RouteRunner(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11c_route_runner")

        self.route_file = Path(str(self.declare_parameter("route_file", "").value))
        self.auto_start = bool(self.declare_parameter("auto_start", True).value)
        self.startup_delay_s = float(self.declare_parameter("startup_delay_s", 7.0).value)
        self.server_timeout_s = float(self.declare_parameter("server_timeout_s", 20.0).value)
        self.segment_timeout_s = float(self.declare_parameter("segment_timeout_s", 45.0).value)
        self.planner_id = str(self.declare_parameter("planner_id", "GridBased").value)
        self.controller_id = str(self.declare_parameter("controller_id", "FollowPath").value)

        if not self.route_file.is_file():
            raise RuntimeError(f"route file does not exist: {self.route_file}")
        self.route = self._load_route(self.route_file)
        self.route_id = str(self.route["route_id"])
        self.map_id = str(self.route["map_id"])
        self.map_hash = str(self.route["map_hash"])
        self.frame_id = str(self.route["frame_id"])
        self.points = list(self.route["points"])

        self.latest_localization: LocalizationStatus | None = None
        self.running = False
        self.start_requested = False
        self.state = "IDLE"
        self.reached_segments = 0
        self.last_message = "route loaded"

        self.runtime_path_pub = self.create_publisher(
            NavPath, "/agt/navigation/runtime_path", TRANSIENT_QOS
        )
        self.state_pub = self.create_publisher(
            String, "/agt/simulation/navigation/route_state", TRANSIENT_QOS
        )
        self.create_subscription(
            LocalizationStatus, "/agt/localization/status", self._on_localization, 20
        )
        self.create_service(
            Trigger,
            "/agt/simulation/navigation/start_route",
            self._on_start_route,
        )
        self.planner_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self.controller_client = ActionClient(self, FollowPath, "/follow_path")

        self._publish_runtime_path()
        self._publish_state()

    @staticmethod
    def _load_route(path: Path) -> dict[str, Any]:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or data.get("schema") != "agt_route/v1":
            raise RuntimeError("route must use schema agt_route/v1")
        for key in ("route_id", "map_id", "map_hash", "frame_id", "points"):
            if key not in data:
                raise RuntimeError(f"route missing field: {key}")
        if data["frame_id"] != "map":
            raise RuntimeError("V25-11C route frame must be map")
        points = data["points"]
        if not isinstance(points, list) or len(points) < 2:
            raise RuntimeError("route must contain at least two points")
        for index, point in enumerate(points):
            if not isinstance(point, dict) or "x" not in point or "y" not in point:
                raise RuntimeError(f"route point {index} must contain x and y")
            for key in ("x", "y", "yaw"):
                if key in point and not math.isfinite(float(point[key])):
                    raise RuntimeError(f"route point {index} has non-finite {key}")
        return data

    def _on_localization(self, msg: LocalizationStatus) -> None:
        self.latest_localization = msg

    def _localization_ready(self) -> bool:
        status = self.latest_localization
        return bool(
            status is not None
            and status.state == LocalizationStatus.STATE_TRACKING
            and status.localization_accepted
            and status.pose_valid
            and status.correction_generation >= 1
            and status.map_id == self.map_id
            and status.map_hash == self.map_hash
        )

    def _pose(self, point: dict[str, Any]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point["x"])
        pose.pose.position.y = float(point["y"])
        qx, qy, qz, qw = yaw_quaternion(float(point.get("yaw", 0.0)))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _publish_runtime_path(self) -> None:
        msg = NavPath()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = [self._pose(point) for point in self.points]
        for pose in msg.poses:
            pose.header.stamp = msg.header.stamp
        self.runtime_path_pub.publish(msg)

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "gate": "V25-11C",
                "route_id": self.route_id,
                "map_id": self.map_id,
                "map_hash": self.map_hash,
                "state": self.state,
                "reached_segments": self.reached_segments,
                "total_segments": len(self.points) - 1,
                "message": self.last_message,
            },
            separators=(",", ":"),
        )
        self.state_pub.publish(msg)

    def _on_start_route(self, _request, response):
        if self.running:
            response.success = False
            response.message = "route execution already running"
        else:
            self.start_requested = True
            response.success = True
            response.message = "route execution requested"
        return response

    def _wait_future(self, future, timeout_s: float, label: str):
        if not spin_until(self, future.done, timeout_s):
            raise RuntimeError(f"timeout waiting for {label}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def _plan(self, start_point, goal_point) -> NavPath:
        goal = ComputePathToPose.Goal()
        goal.start = self._pose(start_point)
        goal.goal = self._pose(goal_point)
        goal.planner_id = self.planner_id
        goal.use_start = True
        handle = self._wait_future(
            self.planner_client.send_goal_async(goal),
            self.server_timeout_s,
            "planner goal response",
        )
        if not handle.accepted:
            raise RuntimeError("ComputePathToPose rejected")
        result = self._wait_future(
            handle.get_result_async(), self.segment_timeout_s, "planner result"
        )
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"planner failed status={result.status}")
        if len(result.result.path.poses) < 2:
            raise RuntimeError("planner returned degenerate path")
        return result.result.path

    def _follow(self, path: NavPath) -> None:
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = "goal_checker"
        handle = self._wait_future(
            self.controller_client.send_goal_async(goal),
            self.server_timeout_s,
            "controller goal response",
        )
        if not handle.accepted:
            raise RuntimeError("FollowPath rejected")
        result = self._wait_future(
            handle.get_result_async(), self.segment_timeout_s, "controller result"
        )
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"controller failed status={result.status}")

    def run_route(self) -> None:
        if self.running:
            return
        self.running = True
        self.state = "RUNNING"
        self.reached_segments = 0
        self.last_message = "waiting for canonical localization and Nav2"
        self._publish_state()
        try:
            if not spin_until(self, self._localization_ready, self.server_timeout_s):
                status = self.latest_localization
                identity = None if status is None else (status.map_id, status.map_hash)
                raise RuntimeError(
                    "canonical localization is not TRACKING on route map; "
                    f"expected=({self.map_id}, {self.map_hash}) latest={identity}"
                )
            if not self.planner_client.wait_for_server(timeout_sec=self.server_timeout_s):
                raise RuntimeError("/compute_path_to_pose unavailable")
            if not self.controller_client.wait_for_server(timeout_sec=self.server_timeout_s):
                raise RuntimeError("/follow_path unavailable")

            for index in range(1, len(self.points)):
                self.last_message = f"planning segment {index}/{len(self.points)-1}"
                self._publish_state()
                path = self._plan(self.points[index - 1], self.points[index])
                self.last_message = f"following segment {index}/{len(self.points)-1}"
                self._publish_state()
                self._follow(path)
                self.reached_segments = index
                self.last_message = f"completed segment {index}/{len(self.points)-1}"
                self._publish_state()

            self.state = "SUCCEEDED"
            self.last_message = "route execution completed"
            self._publish_state()
        except Exception as error:  # noqa: BLE001
            self.state = "FAILED"
            self.last_message = str(error)
            self._publish_state()
            self.get_logger().error(str(error))
        finally:
            self.running = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteRunner()
    auto_deadline = time.monotonic() + node.startup_delay_s
    auto_requested = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.auto_start and not auto_requested and time.monotonic() >= auto_deadline:
                node.start_requested = True
                auto_requested = True
            if node.start_requested and not node.running:
                node.start_requested = False
                node.run_route()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
