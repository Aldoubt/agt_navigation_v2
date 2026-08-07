#!/usr/bin/env python3
"""Test-only capability servers for the V25-07 BT integration test.

This module deliberately contains no production executable entry point.  The
launch test starts it as a temporary Python process and uses it only below the
project Action/Service boundaries.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import threading
import time

import rclpy
from agt_interfaces.action import ExecuteWaypointTask, Relocalize
from agt_interfaces.srv import EvaluateTaskReadiness
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from agt_navigation.task_group import TaskGroup


class P0CapabilityFakes(Node):
    def __init__(self) -> None:
        super().__init__("p0_bt_capability_fakes")
        self._lock = threading.Lock()
        self._active_task = ""
        self._readiness_calls: dict[str, int] = {}
        self._relocalize_goals = 0
        self._relocalize_cancels = 0
        self._waypoint_goals = 0
        self._waypoint_cancels = 0
        self._events: list[str] = []
        self._validation_errors: list[str] = []
        self._last_relocalize_goal = {}
        self._last_waypoint_goal = {}
        self._expected_hashes = {}
        if len(sys.argv) > 1:
            for path in Path(sys.argv[1]).glob("maps/*/versions/*/tasks/*.json"):
                task = TaskGroup.from_json(path)
                self._expected_hashes[task.task_group_id] = task.canonical_hash()
        self._cancel_events: dict[str, threading.Event] = {}
        self._evidence_publisher = self.create_publisher(
            String, "/agt/test/p0_bt/capability_evidence", 10
        )
        self.create_service(
            EvaluateTaskReadiness,
            "/agt/system/evaluate_task_readiness",
            self._readiness,
        )
        self._relocalize = ActionServer(
            self, Relocalize, "/agt/localization/relocalize",
            execute_callback=self._relocalize_execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=self._cancel,
        )
        self._waypoint = ActionServer(
            self, ExecuteWaypointTask, "/agt/navigation/execute_waypoint_task",
            execute_callback=self._waypoint_execute,
            goal_callback=self._waypoint_goal,
            cancel_callback=self._cancel,
        )

    def _scenario(self) -> str:
        return self._active_task.removeprefix("p0_")

    def _publish_evidence(self) -> None:
        with self._lock:
            payload = {
                "readiness_calls": dict(self._readiness_calls),
                "relocalize_goal_count": self._relocalize_goals,
                "relocalize_cancel_count": self._relocalize_cancels,
                "waypoint_goal_count": self._waypoint_goals,
                "waypoint_cancel_count": self._waypoint_cancels,
                "last_relocalize_goal": self._last_relocalize_goal,
                "last_waypoint_goal": self._last_waypoint_goal,
                "events": list(self._events),
                "validation_errors": list(self._validation_errors),
            }
        self._evidence_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _readiness(self, request, response):
        task = str(request.task_id)
        with self._lock:
            if task != self._active_task:
                self._readiness_calls = {}
                self._relocalize_goals = 0
                self._relocalize_cancels = 0
                self._waypoint_goals = 0
                self._waypoint_cancels = 0
                self._events = []
                self._validation_errors = []
                self._last_relocalize_goal = {}
                self._last_waypoint_goal = {}
            self._active_task = task
            index = self._readiness_calls.get(task, 0)
            self._readiness_calls[task] = index + 1
        scenario = task.removeprefix("p0_")
        profile = int(request.gate_profile)
        ready = True
        blocker = ""
        if scenario == "preflight_failure":
            ready, blocker = False, "SENSOR_INPUT_UNHEALTHY"
        elif scenario in {
            "lost_localization", "relocalize_failure", "post_relocalization",
            "cancel_relocalize",
        }:
            if profile == EvaluateTaskReadiness.Request.PROFILE_TASK_EXECUTION:
                ready = scenario == "lost_localization" and index >= 2
                if not ready:
                    blocker = "LOCALIZATION_NOT_TRACKING"
        response.readiness.ready = ready
        response.readiness.map_id = "map_demo"
        response.readiness.map_version_id = "v1"
        response.readiness.localization_state = "TRACKING" if ready else "LOST"
        response.readiness.blocker_codes = [] if ready else [blocker]
        response.readiness.blocker_messages = [] if ready else [blocker]
        self._publish_evidence()
        return response

    def _waypoint_goal(self, goal) -> GoalResponse:
        expected_hash = self._expected_hashes.get(str(goal.task_group_id), "")
        fields = {
            "map_id": str(goal.map_id),
            "map_version_id": str(goal.map_version_id),
            "task_group_id": str(goal.task_group_id),
            "task_revision": int(goal.task_revision),
            "expected_content_sha256": str(goal.expected_content_sha256),
            "loop_count": int(goal.loop_count),
            "client_request_id": str(goal.client_request_id),
            "task_file": str(goal.task_file),
            "poses_count": len(goal.poses),
        }
        errors = []
        if fields["expected_content_sha256"] != expected_hash:
            errors.append("expected_content_sha256 does not equal TaskGroup canonical hash")
        if (
            goal.task_file or goal.poses or goal.map_id != "map_demo"
            or goal.map_version_id != "v1" or not goal.task_group_id
            or goal.task_revision <= 0 or not goal.expected_content_sha256.startswith("sha256:")
            or goal.loop_count <= 0 or not goal.client_request_id.startswith("bt_")
        ):
            errors.append("formal waypoint goal identity/shape contract failed")
        with self._lock:
            self._last_waypoint_goal = fields
            self._validation_errors.extend(errors)
            self._events.append("waypoint_goal_rejected" if errors else "waypoint_goal_accepted")
        self._publish_evidence()
        if errors:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _event_for(self, handle) -> threading.Event:
        event = threading.Event()
        with self._lock:
            self._cancel_events[str(handle.goal_id.uuid)] = event
        return event

    def _cancel(self, handle) -> CancelResponse:
        key = str(handle.goal_id.uuid)
        with self._lock:
            event = self._cancel_events.get(key)
            if event is not None:
                event.set()
            if self._scenario() == "cancel_relocalize":
                self._relocalize_cancels += 1
                self._events.append("relocalize_cancel")
            else:
                self._waypoint_cancels += 1
                self._events.append("waypoint_cancel")
        self._publish_evidence()
        return CancelResponse.ACCEPT

    def _finish_event(self, handle) -> None:
        with self._lock:
            self._cancel_events.pop(str(handle.goal_id.uuid), None)

    def _relocalize_execute(self, handle):
        with self._lock:
            self._relocalize_goals += 1
            scenario = self._scenario()
            self._last_relocalize_goal = {
                "mode": int(handle.request.mode),
                "timeout_s": float(handle.request.timeout_s),
            }
            self._events.append("relocalize_goal")
        self._publish_evidence()
        event = self._event_for(handle)
        if scenario == "cancel_relocalize":
            while not event.wait(0.02) and rclpy.ok():
                pass
        if event.is_set() or handle.is_cancel_requested:
            result = Relocalize.Result(success=False, failure_reason="canceled")
            handle.canceled()
        else:
            result = Relocalize.Result(success=scenario != "relocalize_failure")
            if not result.success:
                result.failure_reason = "fake relocalization failed"
            handle.succeed() if result.success else handle.abort()
        self._finish_event(handle)
        return result

    def _waypoint_execute(self, handle):
        with self._lock:
            self._waypoint_goals += 1
            scenario = self._scenario()
            self._events.append("waypoint_goal")
        self._publish_evidence()
        event = self._event_for(handle)
        feedback = ExecuteWaypointTask.Feedback()
        feedback.state = "RUNNING"
        feedback.loop_index = 0
        feedback.current_waypoint = 2
        feedback.total_waypoints = 5
        handle.publish_feedback(feedback)
        # Leave one BT tick for feedback to cross the child -> BT -> Mission
        # chain before returning the terminal result.
        if scenario not in {"cancel_waypoint"}:
            time.sleep(0.1)
        if scenario in {"cancel_waypoint"}:
            while not event.wait(0.02) and rclpy.ok():
                pass
        if event.is_set() or handle.is_cancel_requested:
            result = ExecuteWaypointTask.Result(success=False, message="canceled")
            handle.canceled()
        else:
            result = ExecuteWaypointTask.Result(
                success=scenario != "waypoint_failure", message="fake waypoint result"
            )
            if result.success:
                handle.succeed()
            else:
                result.error_code = 77
                handle.abort()
        self._finish_event(handle)
        return result


def main() -> None:
    rclpy.init()
    node = P0CapabilityFakes()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
