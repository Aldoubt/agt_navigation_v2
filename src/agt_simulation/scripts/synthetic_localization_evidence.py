#!/usr/bin/env python3

"""SOFTWARE_ONLY Gazebo truth -> V25-10 LocalizationStatus evidence adapter.

This node never publishes TF and never owns canonical localization status. It only
submits sparse localization evidence to the existing GlobalCorrectionManager.
Bias / quality parameters are intentionally mutable so V25-11 can inject
translation, yaw, quality and health-state faults without creating a competing
localization implementation.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from agt_interfaces.msg import LocalizationStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_quaternion(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class SyntheticLocalizationEvidence(Node):
    def __init__(self) -> None:
        super().__init__("agt_synthetic_localization_evidence")

        self.truth_topic = str(
            self.declare_parameter(
                "truth_topic", "/simulation/bunker/ground_truth"
            ).value
        )
        self.evidence_topic = str(
            self.declare_parameter(
                "evidence_topic", "/agt/localization/evidence_status"
            ).value
        )
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.map_id = str(
            self.declare_parameter("map_id", "v25_11_gazebo").value
        )
        self.map_hash = str(
            self.declare_parameter(
                "map_hash", "software-only:agri-validation-v1"
            ).value
        )

        # Mutable fault-injection controls. Values are read at submission time.
        self.declare_parameter("translation_bias_x_m", 0.0)
        self.declare_parameter("translation_bias_y_m", 0.0)
        self.declare_parameter("yaw_bias_deg", 0.0)
        self.declare_parameter("fitness_score", 0.01)
        self.declare_parameter("translation_innovation_m", 0.0)
        self.declare_parameter("yaw_innovation_deg", 0.0)

        self.auto_initial_correction = bool(
            self.declare_parameter("auto_initial_correction", True).value
        )
        self.initial_delay_s = float(
            self.declare_parameter("initial_delay_s", 2.0).value
        )

        self.latest_truth: Optional[Odometry] = None
        self.initial_submitted = False
        self.publisher = self.create_publisher(
            LocalizationStatus, self.evidence_topic, 10
        )
        self.create_subscription(Odometry, self.truth_topic, self._on_truth, 20)

        self.create_service(
            Trigger,
            "/agt/simulation/localization/submit_correction",
            self._submit_correction_service,
        )
        self.create_service(
            Trigger,
            "/agt/simulation/localization/publish_recovering",
            self._publish_recovering_service,
        )
        self.create_service(
            Trigger,
            "/agt/simulation/localization/publish_lost",
            self._publish_lost_service,
        )
        self.create_service(
            Trigger,
            "/agt/simulation/localization/clear_faults",
            self._clear_faults_service,
        )

        self.start_wall_ns = self.get_clock().now().nanoseconds
        self.create_timer(0.10, self._maybe_submit_initial)
        self.get_logger().info(
            "SOFTWARE_ONLY synthetic localization evidence ready: "
            f"truth={self.truth_topic} evidence={self.evidence_topic} map={self.map_id}"
        )

    def _on_truth(self, message: Odometry) -> None:
        self.latest_truth = message

    def _maybe_submit_initial(self) -> None:
        if (
            not self.auto_initial_correction
            or self.initial_submitted
            or self.latest_truth is None
        ):
            return
        elapsed = (
            self.get_clock().now().nanoseconds - self.start_wall_ns
        ) / 1.0e9
        if elapsed < self.initial_delay_s:
            return
        if self._publish_correction():
            self.initial_submitted = True

    def _publish_correction(self) -> bool:
        truth = self.latest_truth
        if truth is None:
            self.get_logger().warning("Cannot submit correction: no Gazebo truth yet")
            return False

        message = LocalizationStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.state = LocalizationStatus.STATE_TRACKING
        message.pose_valid = True
        message.localization_accepted = True
        message.has_converged = True
        message.ambiguous_result = False
        message.status_stale = False
        message.error_code = LocalizationStatus.ERROR_NONE
        message.backend = "gazebo_ground_truth_synthetic"
        message.candidate_source = "SOFTWARE_ONLY_GAZEBO_TRUTH"
        message.candidate_id = "gazebo_truth"
        message.map_id = self.map_id
        message.map_hash = self.map_hash
        # Evidence never owns canonical correction generation.
        message.correction_generation = 0

        message.global_pose.header.stamp = truth.header.stamp
        message.global_pose.header.frame_id = self.map_frame
        message.global_pose.pose.pose.position.x = (
            float(truth.pose.pose.position.x)
            + float(self.get_parameter("translation_bias_x_m").value)
        )
        message.global_pose.pose.pose.position.y = (
            float(truth.pose.pose.position.y)
            + float(self.get_parameter("translation_bias_y_m").value)
        )
        message.global_pose.pose.pose.position.z = 0.0

        q = truth.pose.pose.orientation
        truth_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        yaw = truth_yaw + math.radians(
            float(self.get_parameter("yaw_bias_deg").value)
        )
        qx, qy, qz, qw = _yaw_quaternion(yaw)
        message.global_pose.pose.pose.orientation.x = qx
        message.global_pose.pose.pose.orientation.y = qy
        message.global_pose.pose.pose.orientation.z = qz
        message.global_pose.pose.pose.orientation.w = qw
        message.global_pose.pose.covariance[0] = 0.0025
        message.global_pose.pose.covariance[7] = 0.0025
        message.global_pose.pose.covariance[35] = 0.0012

        message.fitness_score = float(self.get_parameter("fitness_score").value)
        message.overlap_ratio = 1.0
        message.inlier_ratio = 1.0
        message.ambiguity_score = 0.0
        message.translation_innovation = float(
            self.get_parameter("translation_innovation_m").value
        )
        message.yaw_innovation = math.radians(
            float(self.get_parameter("yaw_innovation_deg").value)
        )
        message.runtime_ms = 0.0
        message.tested_candidates = 1
        message.total_candidates = 1
        message.consecutive_successes = 1
        message.consecutive_failures = 0
        message.message = "SOFTWARE_ONLY Gazebo truth correction evidence"
        self.publisher.publish(message)
        return True

    def _publish_health(self, state: int, error_code: int, text: str) -> None:
        message = LocalizationStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.state = state
        message.pose_valid = False
        message.localization_accepted = False
        message.has_converged = False
        message.status_stale = state == LocalizationStatus.STATE_LOST
        message.error_code = error_code
        message.backend = "gazebo_ground_truth_synthetic"
        message.candidate_source = "SOFTWARE_ONLY_FAULT_INJECTION"
        message.map_id = self.map_id
        message.map_hash = self.map_hash
        message.correction_generation = 0
        message.consecutive_failures = 1
        message.message = text
        self.publisher.publish(message)

    def _submit_correction_service(self, _request, response):
        response.success = self._publish_correction()
        response.message = (
            "submitted synthetic correction evidence"
            if response.success
            else "no Gazebo truth available"
        )
        return response

    def _publish_recovering_service(self, _request, response):
        self._publish_health(
            LocalizationStatus.STATE_RECOVERING,
            LocalizationStatus.ERROR_TIMEOUT,
            "SOFTWARE_ONLY injected localization recovering",
        )
        response.success = True
        response.message = "published RECOVERING evidence"
        return response

    def _publish_lost_service(self, _request, response):
        self._publish_health(
            LocalizationStatus.STATE_LOST,
            LocalizationStatus.ERROR_TIMEOUT,
            "SOFTWARE_ONLY injected localization lost",
        )
        response.success = True
        response.message = "published LOST evidence"
        return response

    def _clear_faults_service(self, _request, response):
        self.set_parameters(
            [
                Parameter("translation_bias_x_m", value=0.0),
                Parameter("translation_bias_y_m", value=0.0),
                Parameter("yaw_bias_deg", value=0.0),
                Parameter("fitness_score", value=0.01),
                Parameter("translation_innovation_m", value=0.0),
                Parameter("yaw_innovation_deg", value=0.0),
            ]
        )
        response.success = True
        response.message = "cleared synthetic localization fault parameters"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticLocalizationEvidence()
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
