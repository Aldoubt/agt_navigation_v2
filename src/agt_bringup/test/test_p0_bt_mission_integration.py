#!/usr/bin/env python3
"""V25-07 real Mission -> BT -> project capability integration acceptance."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import tempfile
import time
import unittest

import launch
from launch.actions import EmitEvent, TimerAction
from launch.events import Shutdown
from launch_ros.actions import Node as LaunchNode
import launch_testing
from agt_interfaces.action import ExecuteMission
from agt_interfaces.msg import MissionStatus
from agt_mission_manager.mission_schema import canonical_hash
from agt_navigation.task_group import MapBinding, TaskGroup, Waypoint
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RosNode


MAP_ID = "map_demo"
MAP_VERSION = "v1"
MANIFEST = "sha256:" + "a" * 64
RUNTIME_ROOT: Path | None = None


def _write_fixture(root: Path, scenario: str) -> None:
    map_root = root / "maps" / MAP_ID / "versions" / MAP_VERSION / "tasks"
    map_root.mkdir(parents=True, exist_ok=True)
    binding = MapBinding(
        map_id=MAP_ID, map_version_id=MAP_VERSION, resolution=0.05,
        width=20, height=20, origin=(0.0, 0.0, 0.0),
    )
    task = TaskGroup(
        task_group_id=f"p0_{scenario}", name="P0 test task", description="",
        created_at="2026-01-01T00:00:00Z", updated_at="2026-01-01T00:00:00Z",
        map_binding=binding,
        points=[Waypoint("wp_1", "one", 1.0, 0.0, 0.0)], revision=1,
    )
    task.content_sha256 = task.canonical_hash()
    (map_root / f"p0_{scenario}.json").write_text(
        json.dumps(task.to_dict()), encoding="utf-8"
    )
    document = {
        "schema_version": 1, "mission_id": f"p0_{scenario}",
        "mission_version": "v1",
        "map_binding": {"map_id": MAP_ID, "map_version_id": MAP_VERSION, "manifest_sha256": MANIFEST},
        "steps": [{"id": "waypoints", "type": "WAYPOINT_TASK", "task_file": f"tasks/p0_{scenario}.json"}],
    }
    document["content_sha256"] = canonical_hash(document)
    mission_dir = root / "missions" / f"p0_{scenario}" / "v1"
    mission_dir.mkdir(parents=True)
    import yaml
    (mission_dir / "mission.yaml").write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _spin(executor, future, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert future.done(), "bounded ROS future timed out"
    return future.result()


def generate_test_description():
    global RUNTIME_ROOT
    root = Path(tempfile.mkdtemp(prefix="agt_v25_07_"))
    RUNTIME_ROOT = root
    for scenario in (
        "already_localized", "lost_localization", "preflight_failure",
        "relocalize_failure", "post_relocalization", "waypoint_failure",
        "cancel_waypoint", "cancel_relocalize", "feedback",
    ):
        _write_fixture(root, scenario)
    fake = launch.actions.ExecuteProcess(
        cmd=["python3", str(Path(__file__).with_name("p0_bt_fake_nodes.py"))], output="screen",
        sigterm_timeout="1.0", sigkill_timeout="1.0",
    )
    mission = LaunchNode(
        package="agt_mission_manager", executable="mission_manager_node.py",
        name="agt_mission_manager", output="screen",
        sigterm_timeout="1.0", sigkill_timeout="1.0",
        parameters=[{"runtime_dir": str(root), "execution_backend": "behavior_tree",
                     "waypoint_server_wait_timeout_s": 3.0, "bt_result_timeout_s": 15.0}],
    )
    bt = LaunchNode(
        package="agt_bt_executor", executable="bt_executor_node",
        name="agt_bt_executor", output="screen",
        sigterm_timeout="1.0", sigkill_timeout="1.0",
        parameters=[{"waypoint_timeout_s": 12.0, "relocalize_timeout_s": 12.0}],
    )
    return launch.LaunchDescription([
        fake, mission, bt,
        launch_testing.actions.ReadyToTest(),
        TimerAction(
            period=20.0,
            actions=[EmitEvent(event=Shutdown(reason="P0 integration suite deadline"))],
        ),
    ]), {"runtime_root": root}


class TestP0BTMissionIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RosNode("p0_bt_test_client")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()
        if RUNTIME_ROOT is not None:
            shutil.rmtree(RUNTIME_ROOT, ignore_errors=True)

    def _run(self, scenario, *, cancel=False):
        node, executor = self.node, self.executor
        client = ActionClient(node, ExecuteMission, "/agt/missions/execute")
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))
        request = ExecuteMission.Goal()
        request.mission_id, request.mission_version = f"p0_{scenario}", "v1"
        feedback = []
        send = client.send_goal_async(request, feedback_callback=lambda message: feedback.append(message.feedback.status))
        handle = _spin(executor, send)
        self.assertTrue(handle.accepted)
        if cancel:
            time.sleep(0.2)
            _spin(executor, handle.cancel_goal_async())
        wrapped = _spin(executor, handle.get_result_async(), 20.0)
        return wrapped.result, feedback

    def test_full_chain_success_and_feedback(self):
        result, feedback = self._run("already_localized")
        self.assertTrue(result.success, f"{result.error_code}: {result.message}; {result.final_status.message}")
        self.assertEqual(result.final_status.state, MissionStatus.STATE_SUCCEEDED)
        self.assertTrue(
            any(s.current_waypoint == 2 and s.total_waypoints == 5 for s in feedback),
            repr([(s.current_waypoint, s.total_waypoints, s.message) for s in feedback]),
        )
        self.assertIn("bt_backend_started", Path(result.audit_log_uri).read_text())
        audit = Path(result.audit_log_uri).read_text()
        self.assertTrue(all(event in audit for event in ("bt_tree_started", "bt_tree_succeeded")))

    def _assert_failure(self, scenario):
        result, _ = self._run(scenario)
        self.assertFalse(result.success, result.message)
        if scenario == "relocalize_failure":
            self.assertEqual(result.error_code, MissionStatus.ERROR_CHILD_FAILED)
            self.assertNotEqual(result.error_code, MissionStatus.ERROR_READINESS_LOST)
        if scenario == "preflight_failure":
            self.assertIn("SENSOR_INPUT_UNHEALTHY", result.final_status.blocker_codes)
        if scenario == "post_relocalization":
            self.assertIn("LOCALIZATION_NOT_TRACKING", result.final_status.blocker_codes)

    def test_lost_localization(self):
        result, _ = self._run("lost_localization")
        self.assertTrue(result.success, result.message)
    def test_preflight_failure(self): self._assert_failure("preflight_failure")
    def test_relocalize_failure(self): self._assert_failure("relocalize_failure")
    def test_post_relocalization_failure(self): self._assert_failure("post_relocalization")
    def test_waypoint_failure(self): self._assert_failure("waypoint_failure")

    def _assert_cancel(self, scenario):
        result, _ = self._run(scenario, cancel=True)
        self.assertFalse(result.success)
        self.assertEqual(result.final_status.state, MissionStatus.STATE_CANCELED)

    def test_parent_cancel_during_waypoint(self): self._assert_cancel("cancel_waypoint")
    def test_parent_cancel_during_relocalize(self): self._assert_cancel("cancel_relocalize")


    def test_mission_status_has_single_owner(self):
        deadline = time.monotonic() + 5.0
        publishers = []
        while time.monotonic() < deadline:
            publishers = self.node.get_publishers_info_by_topic("/agt/missions/status")
            if publishers:
                break
            self.executor.spin_once(timeout_sec=0.1)
        self.assertEqual(len(publishers), 1)
        self.assertEqual(publishers[0].node_name, "agt_mission_manager")
