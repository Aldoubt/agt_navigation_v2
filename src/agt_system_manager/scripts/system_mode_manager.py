#!/usr/bin/env python3

import asyncio
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
from agt_interfaces.action import ChangeSystemMode
from agt_interfaces.msg import SystemHealth
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.parameter import Parameter

from agt_system_manager.process_manager import ProcessManager, ProfileRegistry, ProfileError


class SystemModeManager(Node):
    def __init__(self) -> None:
        super().__init__("agt_system_mode_manager")
        default_profiles = str(Path(get_package_share_directory("agt_system_manager")) / "config" / "mode_profiles.yaml")
        profiles_path = str(self.declare_parameter("profiles_file", default_profiles).value)
        runtime_dir = str(self.declare_parameter("runtime_dir", "runtime").value)
        with open(profiles_path, "r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        self._registry = ProfileRegistry(raw.get("profiles", {}), tuple(raw.get("allowed_executables", ["ros2", "rviz2"])))
        self._manager = ProcessManager(self._registry, runtime_dir)
        health_node = str(self.declare_parameter("health_node_name", "agt_system_manager_health").value)
        self._health_parameters = self.create_client(SetParameters, f"/{health_node}/set_parameters")
        self._health_snapshot = None
        self.create_subscription(SystemHealth, "/agt/system/health", self._health_callback, 10)
        self._server = ActionServer(
            self,
            ChangeSystemMode,
            "/agt/system/change_mode",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
        )

    def _health_callback(self, message: SystemHealth) -> None:
        self._health_snapshot = message

    def _set_health_mode(self, mode: str) -> bool:
        if not self._health_parameters.service_is_ready():
            return False
        request = SetParameters.Request()
        request.parameters = [Parameter("active_mode", Parameter.Type.STRING, mode).to_parameter_msg()]
        self._health_parameters.call_async(request)
        return True

    async def _wait_for_health(self, goal_handle, mode: str, timeout_sec: float) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        mode_set = False
        baseline_revision = self._health_snapshot.revision if self._health_snapshot is not None else -1
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, "mode change canceled"
            if not mode_set:
                mode_set = self._set_health_mode(mode)
            snapshot = self._health_snapshot
            if mode_set and snapshot is not None and snapshot.revision > baseline_revision:
                if snapshot.overall_state in (SystemHealth.STATE_OK, SystemHealth.STATE_WARN):
                    return True, "health conditions satisfied"
                goal_handle.publish_feedback(
                    ChangeSystemMode.Feedback(
                        state="WAITING_FOR_HEALTH",
                        progress=0.5,
                        message="; ".join(snapshot.blocker_messages) or "waiting for required health conditions",
                    )
                )
            await asyncio.sleep(0.1)
        if not mode_set:
            return False, "health node parameter service is unavailable"
        return False, "startup health conditions timed out"

    def _goal(self, goal: ChangeSystemMode.Goal):
        if goal.mode not in {
            ChangeSystemMode.Goal.MODE_IDLE,
            ChangeSystemMode.Goal.MODE_SENSOR_ONLY,
            ChangeSystemMode.Goal.MODE_MAPPING,
            ChangeSystemMode.Goal.MODE_LOCALIZATION_DEBUG,
            ChangeSystemMode.Goal.MODE_NAVIGATION,
            ChangeSystemMode.Goal.MODE_ERROR,
        }:
            return GoalResponse.REJECT
        if len(goal.argument_keys) != len(goal.argument_values):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _mode_name(value: int) -> str:
        return {
            ChangeSystemMode.Goal.MODE_IDLE: "IDLE",
            ChangeSystemMode.Goal.MODE_SENSOR_ONLY: "SENSOR_ONLY",
            ChangeSystemMode.Goal.MODE_MAPPING: "MAPPING",
            ChangeSystemMode.Goal.MODE_LOCALIZATION_DEBUG: "LOCALIZATION_DEBUG",
            ChangeSystemMode.Goal.MODE_NAVIGATION: "NAVIGATION",
            ChangeSystemMode.Goal.MODE_ERROR: "ERROR",
        }[value]

    async def _execute(self, goal_handle):
        result = ChangeSystemMode.Result()
        mode = self._mode_name(goal_handle.request.mode)
        started_mode = None
        feedback = ChangeSystemMode.Feedback(state="STARTING", progress=0.0, message=mode)
        goal_handle.publish_feedback(feedback)
        try:
            if len(goal_handle.request.argument_keys) != len(goal_handle.request.argument_values):
                raise ProfileError("launch argument key/value lengths differ")
            arguments = dict(zip(goal_handle.request.argument_keys, goal_handle.request.argument_values))
            if mode in ("IDLE", "ERROR"):
                stopped = self._manager.stop_all()
                result.success = mode == "IDLE"
                result.error_code = 0 if result.success else 1
                result.active_mode = mode
                result.message = "managed processes stopped" if result.success else "ERROR mode is fail-closed"
                result.process_ids = [int(item["pid"]) for item in stopped]
                self._set_health_mode(mode)
            else:
                profile = goal_handle.request.profile or mode.lower()
                managed = self._manager.start(profile, arguments)
                started_mode = mode
                result.active_mode = mode
                result.profile = managed.profile.profile_id
                result.process_ids = [managed.pid]
                result.log_paths = [str(managed.log_path)]
                wait_timeout = min(max(float(goal_handle.request.startup_timeout_s or 30.0), 0.1), 300.0)
                if goal_handle.request.wait_for_health:
                    healthy, wait_message = await self._wait_for_health(goal_handle, mode, wait_timeout)
                    if not healthy:
                        self._manager.stop_mode(mode)
                        self._set_health_mode("IDLE")
                        result.success = False
                        result.error_code = 3
                        result.active_mode = "IDLE"
                        result.message = wait_message
                        goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
                    else:
                        result.success = True
                        result.error_code = 0
                        result.message = wait_message
                        goal_handle.succeed()
                else:
                    result.success = True
                    result.error_code = 0
                    result.message = "managed launch started; health readiness is reported separately"
                    self._set_health_mode(mode)
                    goal_handle.succeed()
            if mode in ("IDLE", "ERROR"):
                goal_handle.succeed()
        except (ProfileError, OSError, ValueError) as error:
            if started_mode is not None:
                self._manager.stop_mode(started_mode)
                self._set_health_mode("IDLE")
            result.success = False
            result.error_code = 2
            result.active_mode = mode
            result.message = str(error)
            goal_handle.abort()
        feedback = ChangeSystemMode.Feedback(state="FINISHED", progress=1.0, message=result.message)
        goal_handle.publish_feedback(feedback)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._manager.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
