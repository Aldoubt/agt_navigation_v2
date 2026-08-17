#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from agt_route_benchmark.path_io import read_path_csv
from agt_route_benchmark.tracking_io import write_executed_trajectory_csv
from agt_route_benchmark.tracking_metrics import ExecutedPose, compute_tracking_metrics


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RppTrackingSimTrial(Node):
    def __init__(self, reference_csv: Path, executed_csv: Path, report_json: Path, timeout_s: float) -> None:
        super().__init__("agt_route_benchmark_rpp_tracking_sim_trial")
        self.reference_csv = reference_csv.expanduser().resolve()
        self.executed_csv = executed_csv.expanduser().resolve()
        self.report_json = report_json.expanduser().resolve()
        self.timeout_s = float(timeout_s)
        if self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self.reference = read_path_csv(self.reference_csv)
        self.executed: list[ExecutedPose] = []
        self._odom_received = False
        self._client = ActionClient(self, FollowPath, "follow_path")
        self.create_subscription(Odometry, "/agt/mapping/odometry", self._odom_callback, 50)

    def _odom_callback(self, message: Odometry) -> None:
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        pose = message.pose.pose
        self.executed.append(
            ExecutedPose(
                stamp_s=stamp,
                x_m=float(pose.position.x),
                y_m=float(pose.position.y),
                yaw_rad=_yaw_from_quaternion(pose.orientation),
            )
        )
        self._odom_received = True

    def _wait_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return bool(predicate())

    def _nav_path(self) -> NavPath:
        path = NavPath()
        # The lightweight simulation publishes map->odom as identity. Geometry is
        # therefore unchanged; only the frame label is switched to the controller's
        # local-costmap frame so the trial isolates tracking rather than TF errors.
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        for point in self.reference:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point.x_m)
            pose.pose.position.y = float(point.y_m)
            pose.pose.orientation.z = math.sin(float(point.yaw_rad) * 0.5)
            pose.pose.orientation.w = math.cos(float(point.yaw_rad) * 0.5)
            path.poses.append(pose)
        return path

    def run(self) -> dict:
        if not self._wait_until(lambda: self._odom_received, 5.0):
            raise RuntimeError("timed out waiting for Ackermann simulator odometry")
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("timed out waiting for Nav2 FollowPath action")

        goal = FollowPath.Goal()
        goal.path = self._nav_path()
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        send_future = self._client.send_goal_async(goal)
        if not self._wait_until(send_future.done, 10.0):
            raise RuntimeError("timed out waiting for FollowPath goal response")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Nav2 controller_server rejected reference route")

        result_future = handle.get_result_async()
        if not self._wait_until(result_future.done, self.timeout_s):
            handle.cancel_goal_async()
            action_status = -1
            action_error_code = -1
            action_error_msg = "tracking trial timed out"
        else:
            wrapped = result_future.result()
            if wrapped is None:
                action_status = -1
                action_error_code = -1
                action_error_msg = "FollowPath returned no result"
            else:
                action_status = int(wrapped.status)
                result = wrapped.result
                action_error_code = int(getattr(result, "error_code", 0))
                action_error_msg = str(getattr(result, "error_msg", ""))

        # Capture the final stopped pose after controller termination/cancellation.
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.executed:
            raise RuntimeError("no executed trajectory samples were recorded")
        write_executed_trajectory_csv(self.executed, self.executed_csv)
        metrics = compute_tracking_metrics(self.reference, self.executed)
        success = (
            action_status == GoalStatus.STATUS_SUCCEEDED
            and action_error_code == 0
            and metrics["path_completion_ratio"] >= 0.95
            and metrics["final_position_error_m"] <= 0.25
        )
        report = {
            "schema_version": "1.0",
            "success": success,
            "trial_kind": "lightweight_ackermann_rpp",
            "reference_geometry_modified": False,
            "frame_assumption": "map_equals_odom_identity",
            "action_status": action_status,
            "action_error_code": action_error_code,
            "action_error_msg": action_error_msg,
            "reference_path_csv": str(self.reference_csv),
            "executed_trajectory_csv": str(self.executed_csv),
            **metrics,
        }
        self.report_json.parent.mkdir(parents=True, exist_ok=True)
        self.report_json.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        return report


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Track one Paper I path.csv with Nav2 RPP in the lightweight Ackermann simulator")
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--executed", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=120.0)
    known, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = RppTrackingSimTrial(known.reference, known.executed, known.report, known.timeout)
    exit_code = 1
    try:
        report = node.run()
        print(json.dumps(report, indent=2, sort_keys=True))
        exit_code = 0 if report["success"] else 2
    except Exception as exc:
        print(json.dumps({"success": False, "error": str(exc)}, indent=2), flush=True)
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
