#!/usr/bin/env python3

from __future__ import annotations

import copy
from pathlib import Path
import threading
import time

from agt_interfaces.msg import (
    BagSessionSummary, LocalizationStatus, MapVersionSummary, MissionStatus, RobotState,
    SystemHealth, TaskReadiness,
)
from agt_interfaces.srv import GetRobotState
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from agt_system_manager.robot_state import (
    load_process_status, mode_value, nav2_state_from_health, parse_chassis_status,
    parse_safety_status,
)


class RobotStateAggregator(Node):
    def __init__(self) -> None:
        super().__init__("agt_robot_state_aggregator")
        runtime_dir = Path(str(self.declare_parameter("runtime_dir", "runtime").value)).expanduser()
        self._process_status_path = runtime_dir / "logs" / "system_manager" / "process_status.json"
        self._publish_period_s = float(self.declare_parameter("publish_period_s", 0.5).value)
        self._health_timeout_s = float(self.declare_parameter("health_timeout_s", 3.0).value)
        self._readiness_timeout_s = float(self.declare_parameter("readiness_timeout_s", 3.0).value)
        self._map_timeout_s = float(self.declare_parameter("map_timeout_s", 3.0).value)
        self._mission_timeout_s = float(self.declare_parameter("mission_timeout_s", 3.0).value)
        self._localization_timeout_s = float(self.declare_parameter("localization_timeout_s", 10.0).value)
        self._safety_timeout_s = float(self.declare_parameter("safety_timeout_s", 2.0).value)
        self._chassis_timeout_s = float(self.declare_parameter("chassis_timeout_s", 2.0).value)
        self._bag_timeout_s = float(self.declare_parameter("bag_timeout_s", 3.0).value)
        if min(
            self._publish_period_s, self._health_timeout_s, self._readiness_timeout_s,
            self._map_timeout_s, self._mission_timeout_s, self._localization_timeout_s,
            self._safety_timeout_s, self._chassis_timeout_s, self._bag_timeout_s,
        ) <= 0.0:
            raise ValueError("RobotState periods and freshness windows must be positive")
        self._values = {}
        self._seen = {}
        self._revision = 0
        self._latest = RobotState()
        self._lock = threading.RLock()
        callback_group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(RobotState, "/agt/system/robot_state", latched)
        self.create_service(
            GetRobotState, "/agt/system/get_robot_state", self._get_state, callback_group=callback_group
        )
        subscriptions = (
            ("health", SystemHealth, "/agt/system/health", 10),
            ("readiness", TaskReadiness, "/agt/system/task_readiness", 10),
            ("map", MapVersionSummary, "/agt/maps/active", latched),
            ("localization", LocalizationStatus, "/agt/localization/status", 10),
            ("mission", MissionStatus, "/agt/missions/status", latched),
            ("bag", BagSessionSummary, "/agt/data/bags/status", latched),
            ("safety", DiagnosticArray, "/agt/safety/status", 10),
            ("chassis", DiagnosticArray, "/agt/chassis/status", 10),
            ("odometry", Odometry, "/agt/chassis/odometry", 10),
        )
        for key, message_type, topic, qos in subscriptions:
            self.create_subscription(
                message_type,
                topic,
                lambda message, key=key: self._input(key, message),
                qos,
                callback_group=callback_group,
            )
        self._timer = self.create_timer(
            self._publish_period_s, self._publish, callback_group=callback_group
        )
        self._publish()

    def _input(self, key: str, message) -> None:
        with self._lock:
            self._values[key] = copy.deepcopy(message)
            self._seen[key] = time.monotonic()
        self._publish()

    def _fresh(self, key: str, timeout: float, now: float) -> bool:
        return key in self._values and now - self._seen.get(key, float("-inf")) <= timeout

    def _build(self) -> RobotState:
        now = time.monotonic()
        message = RobotState()
        message.header.stamp = self.get_clock().now().to_msg()
        self._revision += 1
        message.revision = self._revision

        readiness_fresh = self._fresh("readiness", self._readiness_timeout_s, now)
        readiness = self._values.get("readiness") if readiness_fresh else None
        active_mode = str(readiness.active_mode) if readiness is not None else "UNKNOWN"
        message.system_mode = mode_value(active_mode)
        total, running, active_profile = load_process_status(self._process_status_path)
        message.managed_process_count = total
        message.running_process_count = running
        message.active_profile = active_profile

        health_fresh = self._fresh("health", self._health_timeout_s, now)
        message.system_health_known = health_fresh
        message.system_health_freshness_s = (
            max(0.0, now - self._seen["health"]) if health_fresh else float("inf")
        )
        if health_fresh:
            message.system_health = copy.deepcopy(self._values["health"])
        message.task_readiness_known = readiness is not None
        message.task_readiness_freshness_s = (
            max(0.0, now - self._seen["readiness"])
            if readiness is not None else float("inf")
        )
        if readiness is not None:
            message.task_readiness = copy.deepcopy(readiness)
        map_fresh = self._fresh("map", self._map_timeout_s, now)
        message.active_map_known = bool(
            map_fresh and getattr(self._values["map"], "active", False)
        )
        message.active_map_freshness_s = (
            max(0.0, now - self._seen["map"]) if map_fresh else float("inf")
        )
        if map_fresh:
            message.active_map = copy.deepcopy(self._values["map"])
        localization_fresh = self._fresh("localization", self._localization_timeout_s, now)
        message.localization_status_known = localization_fresh
        message.localization_freshness_s = (
            max(0.0, now - self._seen["localization"])
            if localization_fresh else float("inf")
        )
        if localization_fresh:
            message.localization = copy.deepcopy(self._values["localization"])
        mission_fresh = self._fresh("mission", self._mission_timeout_s, now)
        message.mission_status_known = mission_fresh
        message.mission_freshness_s = (
            max(0.0, now - self._seen["mission"]) if mission_fresh else float("inf")
        )
        if mission_fresh:
            message.mission = copy.deepcopy(self._values["mission"])
        bag_fresh = self._fresh("bag", self._bag_timeout_s, now)
        message.bag_status_known = bag_fresh
        message.bag_freshness_s = (
            max(0.0, now - self._seen["bag"]) if bag_fresh else float("inf")
        )
        if bag_fresh:
            message.bag_session = copy.deepcopy(self._values["bag"])

        if health_fresh:
            message.nav2_state = nav2_state_from_health(message.system_health, active_mode)
            message.nav2_freshness_s = max(0.0, now - self._seen["health"])
        else:
            message.nav2_state = RobotState.NAV2_UNKNOWN
            message.nav2_freshness_s = float("inf")

        safety_fresh = self._fresh("safety", self._safety_timeout_s, now)
        safety = parse_safety_status(self._values["safety"]) if safety_fresh else {"known": False}
        message.safety_status_known = bool(safety.get("known", False))
        message.safety_motion_enabled = bool(safety.get("motion_enabled", False))
        message.emergency_stop = bool(safety.get("emergency_stop", False))
        message.estop_latched = bool(safety.get("estop_latched", False))
        message.navigation_ready = bool(safety.get("navigation_ready", False))
        message.safety_freshness_s = max(0.0, now - self._seen["safety"]) if safety_fresh else float("inf")

        chassis_fresh = self._fresh("chassis", self._chassis_timeout_s, now)
        chassis = parse_chassis_status(self._values["chassis"]) if chassis_fresh else {"known": False}
        message.chassis_status_known = bool(chassis.get("known", False))
        message.chassis_connected = bool(chassis.get("connected", False))
        message.chassis_control_mode = int(chassis.get("control_mode", 0))
        message.chassis_status_freshness_s = max(0.0, now - self._seen["chassis"]) if chassis_fresh else float("inf")
        message.chassis_odometry_freshness_s = (
            max(0.0, now - self._seen["odometry"])
            if self._fresh("odometry", self._chassis_timeout_s, now)
            else float("inf")
        )

        blockers = []
        blocker_messages = []
        if message.system_mode == RobotState.MODE_UNKNOWN:
            blockers.append("SYSTEM_MODE_UNKNOWN")
            blocker_messages.append("system mode evidence is missing or stale")
        if readiness is not None:
            blockers.extend(readiness.blocker_codes)
            blocker_messages.extend(readiness.blocker_messages)
        else:
            blockers.append("TASK_READINESS_UNKNOWN")
            blocker_messages.append("TaskReadiness evidence is missing or stale")
        if not message.safety_status_known:
            blockers.append("SAFETY_STATUS_UNKNOWN")
            blocker_messages.append("safety diagnostics are missing or stale")
        if not message.chassis_status_known:
            blockers.append("CHASSIS_STATUS_UNKNOWN")
            blocker_messages.append("chassis diagnostics are missing or stale")
        message.blocker_codes = list(dict.fromkeys(blockers))
        message.blocker_messages = list(dict.fromkeys(blocker_messages))
        message.error_code = 0 if not message.blocker_codes else 1
        message.message = "robot state ready" if not message.blocker_codes else "; ".join(message.blocker_messages)
        return message

    def _publish(self) -> None:
        with self._lock:
            self._latest = self._build()
            message = copy.deepcopy(self._latest)
        self._publisher.publish(message)

    def _get_state(self, _request, response):
        with self._lock:
            response.state = copy.deepcopy(self._latest)
        response.success = True
        response.error_code = 0
        response.message = "current RobotState snapshot"
        return response

    def destroy_node(self):
        self._timer.cancel()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotStateAggregator()
    executor = MultiThreadedExecutor(num_threads=3)
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
