#!/usr/bin/env python3

"""Bounded automatic relocalization coordinator."""

import time

import rclpy
from agt_interfaces.action import Relocalize
from agt_interfaces.msg import LocalizationStatus
from agt_interfaces.srv import SetLocalizationMode
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from agt_system_manager.localization_mode import RelocalizationPolicy


class RelocalizationModeController(Node):
    def __init__(self) -> None:
        super().__init__("agt_relocalization_mode_controller")
        self._policy = RelocalizationPolicy(
            mode=str(self.declare_parameter("mode", "MANUAL_ONLY").value),
            max_attempts=int(self.declare_parameter("max_attempts", 3).value),
            retry_cooldown_s=float(self.declare_parameter("retry_cooldown_s", 10.0).value),
            total_timeout_s=float(self.declare_parameter("total_timeout_s", 60.0).value),
            max_candidates=int(self.declare_parameter("max_candidates", 128).value),
        )
        self._action_name = self.declare_parameter("action_name", "/agt/localization/relocalize").value
        self._cloud_topic = self.declare_parameter("cloud_topic", "/agt/mapping/registered_points_lidar").value
        self._cloud_seen_at = float("-inf")
        self._cloud_timeout = float(self.declare_parameter("cloud_timeout_s", 2.0).value)
        self._status = LocalizationStatus()
        self._action_active = False
        self.create_service(SetLocalizationMode, "/agt/localization/set_mode", self._set_mode)
        self._client = ActionClient(self, Relocalize, self._action_name)
        self.create_subscription(PointCloud2, self._cloud_topic, self._cloud_callback, 10)
        self.create_subscription(LocalizationStatus, "/agt/localization/status", self._status_callback, 10)
        self.create_timer(0.25, self._tick)

    def _cloud_callback(self, _message: PointCloud2) -> None:
        self._cloud_seen_at = time.monotonic()

    def _status_callback(self, message: LocalizationStatus) -> None:
        self._status = message

    @staticmethod
    def _mode_name(value: int) -> str:
        return {0: "MANUAL_ONLY", 1: "AUTO_ON_START", 2: "AUTO_RECOVERY"}[value]

    @staticmethod
    def _mode_value(name: str) -> int:
        return {"MANUAL_ONLY": 0, "AUTO_ON_START": 1, "AUTO_RECOVERY": 2}[name]

    def _set_mode(self, request, response):
        try:
            self._policy.set_mode(self._mode_name(request.mode))
            response.success = True
            response.error_code = 0
            response.active_mode = request.mode
            response.attempts = self._policy.attempts
            response.message = "mode updated; automatic requests remain bounded"
        except (KeyError, ValueError) as error:
            response.success = False
            response.error_code = 1
            response.active_mode = self._mode_value(self._policy.mode)
            response.attempts = self._policy.attempts
            response.message = str(error)
        return response

    def _tick(self) -> None:
        now = time.monotonic()
        cloud_healthy = now - self._cloud_seen_at <= self._cloud_timeout
        map_ready = bool(self._status.map_hash)
        state = {LocalizationStatus.STATE_DEGRADED: "DEGRADED", LocalizationStatus.STATE_RECOVERING: "RECOVERING", LocalizationStatus.STATE_LOST: "LOST"}.get(self._status.state, "OTHER")
        if self._action_active or not self._policy.should_trigger(now=now, localization_state=state, map_ready=map_ready, cloud_healthy=cloud_healthy):
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            return
        self._policy.start_attempt(now)
        self._action_active = True
        goal = Relocalize.Goal()
        goal.mode = Relocalize.Goal.MODE_AUTO_SEARCH
        goal.use_last_valid_pose = True
        goal.use_configured_candidates = True
        goal.use_external_coarse_pose = True
        goal.max_candidates = self._policy.max_candidates
        goal.timeout_s = self._policy.total_timeout_s
        self._client.send_goal_async(goal).add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception:
            self._action_active = False
            self._policy.finish_attempt(False)
            return
        if not handle.accepted:
            self._action_active = False
            self._policy.finish_attempt(False)
            return
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future) -> None:
        try:
            success = bool(future.result().result.success)
        except Exception:
            success = False
        self._policy.finish_attempt(success)
        self._action_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelocalizationModeController()
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
