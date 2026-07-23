#!/usr/bin/env python3

import time

from agt_interfaces.action import Relocalize
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def make_startup_goal(
    timeout_s: float, max_candidates: int, publish_debug: bool
) -> Relocalize.Goal:
    goal = Relocalize.Goal()
    goal.mode = Relocalize.Goal.MODE_AUTO_SEARCH
    goal.use_last_valid_pose = True
    goal.use_configured_candidates = True
    goal.use_external_coarse_pose = True
    goal.max_candidates = max(0, int(max_candidates))
    goal.publish_debug = bool(publish_debug)
    goal.timeout_s = float(timeout_s)
    return goal


class AutomaticRelocalization(Node):
    def __init__(self) -> None:
        super().__init__("agt_automatic_relocalization")
        self._action_name = self.declare_parameter(
            "action_name", "/agt/localization/relocalize"
        ).value
        self._startup_delay_s = float(
            self.declare_parameter("startup_delay_s", 3.0).value
        )
        self._server_wait_timeout_s = float(
            self.declare_parameter("server_wait_timeout_s", 15.0).value
        )
        self._cloud_topic = self.declare_parameter(
            "cloud_topic", "/agt/mapping/registered_points_lidar"
        ).value
        self._action_timeout_s = float(
            self.declare_parameter("action_timeout_s", 30.0).value
        )
        self._max_candidates = int(
            self.declare_parameter("max_candidates", 0).value
        )
        self._publish_debug = bool(
            self.declare_parameter("publish_debug", False).value
        )
        if (
            self._startup_delay_s < 0.0
            or self._server_wait_timeout_s <= 0.0
            or self._action_timeout_s <= 0.0
            or self._max_candidates < 0
        ):
            raise ValueError("automatic relocalization parameters are invalid")

        self._client = ActionClient(self, Relocalize, self._action_name)
        self._cloud_seen = False
        self.create_subscription(
            PointCloud2,
            self._cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self._started = False
        self._ready_deadline = time.monotonic() + self._server_wait_timeout_s
        self._start_after = time.monotonic() + self._startup_delay_s
        self._timer = self.create_timer(0.25, self._tick)

    def _cloud_callback(self, _message: PointCloud2) -> None:
        self._cloud_seen = True

    def _tick(self) -> None:
        if self._started:
            return
        now = time.monotonic()
        if now < self._start_after or not self._cloud_seen:
            if now >= self._ready_deadline:
                self._started = True
                self._timer.cancel()
                self.get_logger().error(
                    "automatic relocalization prerequisites were not ready within %.1f s "
                    "(cloud_seen=%s)",
                    self._server_wait_timeout_s,
                    self._cloud_seen,
                )
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            if now >= self._ready_deadline:
                self._started = True
                self._timer.cancel()
                self.get_logger().error(
                    "relocalization action server did not become ready within %.1f s",
                    self._server_wait_timeout_s,
                )
            return

        self._started = True
        self._timer.cancel()
        goal = make_startup_goal(
            self._action_timeout_s, self._max_candidates, self._publish_debug
        )
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)
        self.get_logger().info(
            "started one-shot automatic relocalization action: %s", self._action_name
        )

    def _goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # keep startup failure visible and fail-closed
            self.get_logger().error("automatic relocalization goal failed: %s", error)
            return
        if not goal_handle.accepted:
            self.get_logger().error("automatic relocalization goal was rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result)

    def _result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:  # keep startup failure visible and fail-closed
            self.get_logger().error("automatic relocalization result failed: %s", error)
            return
        if result.success:
            self.get_logger().info(
                "automatic relocalization accepted: state=%d map_hash=%s",
                result.final_status.state,
                result.final_status.map_hash,
            )
        else:
            self.get_logger().error(
                "automatic relocalization failed: error_code=%d reason=%s",
                result.error_code,
                result.failure_reason,
            )

    def _feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().debug(
            "automatic relocalization progress: %d/%d state=%d",
            feedback.tested_candidates,
            feedback.total_candidates,
            feedback.state,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutomaticRelocalization()
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
