#!/usr/bin/env python3

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from agt_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class Acceptance(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11c_navigation_acceptance")
        self.report_path = Path(
            str(
                self.declare_parameter(
                    "report_path", "/tmp/agt_v25_11c_navigation_result.json"
                ).value
            )
        )
        self.expected_goal_x = float(self.declare_parameter("expected_goal_x", -2.0).value)
        self.expected_goal_y = float(self.declare_parameter("expected_goal_y", 0.0).value)
        self.goal_tolerance_m = float(self.declare_parameter("goal_tolerance_m", 0.55).value)
        self.timeout_s = float(self.declare_parameter("timeout_s", 80.0).value)

        self.map: OccupancyGrid | None = None
        self.runtime_path: NavPath | None = None
        self.localization: LocalizationStatus | None = None
        self.first_truth: Odometry | None = None
        self.latest_truth: Odometry | None = None
        self.route_state: dict | None = None
        self.max_cmd_magnitude = 0.0

        self.create_subscription(OccupancyGrid, "/map", self._on_map, TRANSIENT_QOS)
        self.create_subscription(
            NavPath, "/agt/navigation/runtime_path", self._on_path, TRANSIENT_QOS
        )
        self.create_subscription(
            String,
            "/agt/simulation/navigation/route_state",
            self._on_route_state,
            TRANSIENT_QOS,
        )
        self.create_subscription(
            LocalizationStatus, "/agt/localization/status", self._on_localization, 20
        )
        self.create_subscription(
            Odometry, "/simulation/bunker/ground_truth", self._on_truth, 20
        )
        self.create_subscription(Twist, "/agt/safety/cmd_vel", self._on_cmd, 20)

        self.planner_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.controller_client = ActionClient(self, FollowPath, "/follow_path")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def _on_path(self, msg: NavPath) -> None:
        self.runtime_path = msg

    def _on_route_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"state": "INVALID", "message": msg.data}
        if isinstance(data, dict):
            self.route_state = data

    def _on_localization(self, msg: LocalizationStatus) -> None:
        self.localization = msg

    def _on_truth(self, msg: Odometry) -> None:
        if self.first_truth is None:
            self.first_truth = msg
        self.latest_truth = msg

    def _on_cmd(self, msg: Twist) -> None:
        self.max_cmd_magnitude = max(
            self.max_cmd_magnitude,
            math.hypot(float(msg.linear.x), float(msg.angular.z)),
        )

    def route_finished(self) -> bool:
        return isinstance(self.route_state, dict) and self.route_state.get("state") in {
            "SUCCEEDED",
            "FAILED",
        }

    def goal_distance(self) -> float | None:
        if self.latest_truth is None:
            return None
        p = self.latest_truth.pose.pose.position
        return math.hypot(float(p.x) - self.expected_goal_x, float(p.y) - self.expected_goal_y)

    def displacement(self) -> float | None:
        if self.first_truth is None or self.latest_truth is None:
            return None
        a = self.first_truth.pose.pose.position
        b = self.latest_truth.pose.pose.position
        return math.hypot(float(b.x - a.x), float(b.y - a.y))

    def route_on_free_cells(self) -> bool:
        grid = self.map
        path = self.runtime_path
        if grid is None or path is None or not path.poses:
            return False
        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        if resolution <= 0.0 or width <= 0 or height <= 0:
            return False
        ox = float(grid.info.origin.position.x)
        oy = float(grid.info.origin.position.y)
        for pose in path.poses:
            col = int(math.floor((float(pose.pose.position.x) - ox) / resolution))
            row = int(math.floor((float(pose.pose.position.y) - oy) / resolution))
            if col < 0 or row < 0 or col >= width or row >= height:
                return False
            value = int(grid.data[row * width + col])
            if value < 0 or value >= 65:
                return False
        return True


def spin_until(node: Node, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return bool(predicate())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Acceptance()
    result = {"gate": "V25-11C", "status": "FAIL", "checks": {}, "metrics": {}}
    exit_code = 1

    try:
        spin_until(
            node,
            lambda: node.map is not None
            and node.runtime_path is not None
            and node.localization is not None
            and node.first_truth is not None
            and node.route_state is not None,
            20.0,
        )

        result["checks"]["map_available"] = node.map is not None
        result["checks"]["map_frame_contract"] = (
            node.map is not None and node.map.header.frame_id == "map"
        )
        if node.map is not None:
            values = list(node.map.data)
            result["checks"]["map_has_occupied_and_free_space"] = (
                any(v >= 65 for v in values) and any(0 <= v < 65 for v in values)
            )
            result["metrics"]["map_width"] = int(node.map.info.width)
            result["metrics"]["map_height"] = int(node.map.info.height)
            result["metrics"]["map_resolution_m"] = float(node.map.info.resolution)
        else:
            result["checks"]["map_has_occupied_and_free_space"] = False

        result["checks"]["planner_action_available"] = node.planner_client.wait_for_server(timeout_sec=5.0)
        result["checks"]["controller_action_available"] = node.controller_client.wait_for_server(timeout_sec=5.0)
        result["checks"]["runtime_path_available"] = (
            node.runtime_path is not None and len(node.runtime_path.poses) >= 2
        )
        result["checks"]["runtime_path_map_frame"] = (
            node.runtime_path is not None and node.runtime_path.header.frame_id == "map"
        )
        result["checks"]["runtime_path_on_free_map_cells"] = node.route_on_free_cells()
        result["checks"]["localization_tracking"] = (
            node.localization is not None
            and node.localization.state == LocalizationStatus.STATE_TRACKING
            and node.localization.localization_accepted
            and node.localization.pose_valid
            and node.localization.correction_generation >= 1
        )
        result["checks"]["route_map_identity_matches_localization"] = bool(
            isinstance(node.route_state, dict)
            and node.localization is not None
            and node.route_state.get("map_id") == node.localization.map_id
            and node.route_state.get("map_hash") == node.localization.map_hash
        )

        spin_until(node, node.route_finished, node.timeout_s)
        spin_until(
            node,
            lambda: node.goal_distance() is not None
            and node.goal_distance() <= node.goal_tolerance_m,
            5.0,
        )

        displacement = node.displacement()
        goal_distance = node.goal_distance()
        result["metrics"]["physics_displacement_m"] = displacement
        result["metrics"]["goal_distance_m"] = goal_distance
        result["metrics"]["max_cmd_magnitude"] = node.max_cmd_magnitude
        result["metrics"]["route_state"] = node.route_state

        result["checks"]["route_execution_succeeded"] = bool(
            isinstance(node.route_state, dict)
            and node.route_state.get("state") == "SUCCEEDED"
            and int(node.route_state.get("reached_segments", 0))
            == int(node.route_state.get("total_segments", -1))
        )
        result["checks"]["physics_model_moves"] = displacement is not None and displacement >= 0.50
        result["checks"]["navigation_cmd_published"] = node.max_cmd_magnitude >= 0.05
        result["checks"]["goal_reached"] = goal_distance is not None and goal_distance <= node.goal_tolerance_m

        passed = all(bool(v) for v in result["checks"].values())
        result["status"] = "PASS" if passed else "FAIL"
        exit_code = 0 if passed else 1
    except Exception as error:  # noqa: BLE001
        result["error"] = str(error)
    finally:
        node.report_path.parent.mkdir(parents=True, exist_ok=True)
        node.report_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
