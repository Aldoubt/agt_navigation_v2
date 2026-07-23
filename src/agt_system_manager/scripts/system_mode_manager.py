#!/usr/bin/env python3

from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
from agt_interfaces.action import ChangeSystemMode
from agt_interfaces.msg import SystemHealth
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from agt_system_manager.process_manager import ProcessManager, ProfileRegistry, ProfileError


class SystemModeManager(Node):
    _MAIN_RUNTIME_MODES = {"MAPPING", "NAVIGATION"}

    def __init__(self) -> None:
        super().__init__("agt_system_mode_manager")
        default_profiles = str(Path(get_package_share_directory("agt_system_manager")) / "config" / "mode_profiles.yaml")
        profiles_path = str(self.declare_parameter("profiles_file", default_profiles).value)
        runtime_dir = str(self.declare_parameter("runtime_dir", "runtime").value)
        self._status_path = Path(runtime_dir).expanduser() / "logs" / "system_manager" / "process_status.json"
        with open(profiles_path, "r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        self._registry = ProfileRegistry(raw.get("profiles", {}), tuple(raw.get("allowed_executables", ["ros2", "rviz2"])))
        self._manager = ProcessManager(self._registry, runtime_dir)
        health_node = str(self.declare_parameter("health_node_name", "agt_system_manager_health").value)
        self._health_parameters = self.create_client(SetParameters, f"/{health_node}/set_parameters")
        self._health_snapshot = None
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            SystemHealth,
            "/agt/system/health",
            self._health_callback,
            10,
            callback_group=callback_group,
        )
        self._server = ActionServer(
            self,
            ChangeSystemMode,
            "/agt/system/change_mode",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=callback_group,
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

    def _wait_for_health(self, goal_handle, mode: str, timeout_sec: float) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        mode_set = False
        baseline_revision = self._health_snapshot.revision if self._health_snapshot is not None else -1
        latest_blockers = "waiting for a fresh health snapshot"
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, "mode change canceled"
            if not mode_set:
                mode_set = self._set_health_mode(mode)
            snapshot = self._health_snapshot
            if mode_set and snapshot is not None and snapshot.revision > baseline_revision:
                if snapshot.overall_state in (SystemHealth.STATE_OK, SystemHealth.STATE_WARN):
                    return True, "health conditions satisfied"
                latest_blockers = "; ".join(snapshot.blocker_messages) or "required health conditions are not satisfied"
                goal_handle.publish_feedback(
                    ChangeSystemMode.Feedback(
                        state="WAITING_FOR_HEALTH",
                        progress=0.5,
                        message="; ".join(snapshot.blocker_messages) or "waiting for required health conditions",
                    )
                )
            time.sleep(0.1)
        if not mode_set:
            return False, "health node parameter service is unavailable"
        return False, f"startup health conditions timed out: {latest_blockers}"

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

    def _prepare_main_transition(self, mode: str) -> None:
        """Stop only the previous main chain; SENSOR_ONLY remains alive."""
        modes_to_stop = self._MAIN_RUNTIME_MODES if mode == "SENSOR_ONLY" else {"MAPPING", "NAVIGATION"} - {mode}
        for previous_mode in modes_to_stop:
            if previous_mode != mode:
                self._manager.stop_mode(previous_mode)

    @staticmethod
    def _as_bool(value, default: bool = True) -> bool:
        if value is None:
            return default
        normalized = str(value).strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
        raise ProfileError(f"invalid boolean launch value: {value}")

    def _ensure_separate_sensor(self, arguments: dict[str, str]) -> None:
        """Keep the sensor in its own process group across main-chain switches."""
        if not self._as_bool(arguments.get("start_sensor"), True):
            return
        sensor_running = any(
            item.get("profile") == "sensor_only" and item.get("returncode") is None
            for item in self._manager.status()
        )
        if not sensor_running:
            sensor_arguments = {
                key: arguments[key]
                for key in ("user_config_path", "use_sim_time")
                if key in arguments
            }
            self._manager.start("sensor_only", sensor_arguments)
        arguments["start_sensor"] = "false"

    def _write_status(self) -> None:
        try:
            self._manager.write_status(self._status_path)
        except OSError as error:
            self.get_logger().warning(f"failed to write managed process status: {error}")

    def _execute(self, goal_handle):
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
                self._write_status()
            else:
                profile = goal_handle.request.profile or mode.lower()
                if mode in self._MAIN_RUNTIME_MODES and profile in {"mapping", "navigation"}:
                    self._prepare_main_transition(mode)
                    self._ensure_separate_sensor(arguments)
                elif mode == "SENSOR_ONLY" and profile == "sensor_only":
                    self._prepare_main_transition(mode)
                managed = self._manager.start(profile, arguments)
                started_mode = mode
                result.active_mode = mode
                result.profile = managed.profile.profile_id
                managed_items = self._manager.status()
                result.process_ids = [int(item["pid"]) for item in managed_items]
                result.log_paths = [str(item["log_path"]) for item in managed_items]
                self._write_status()
                wait_timeout = min(max(float(goal_handle.request.startup_timeout_s or 30.0), 0.1), 300.0)
                if goal_handle.request.wait_for_health:
                    healthy, wait_message = self._wait_for_health(goal_handle, mode, wait_timeout)
                    if not healthy:
                        self._manager.stop_mode(mode)
                        self._write_status()
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
            self._write_status()
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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._manager.stop_all()
        node._write_status()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
