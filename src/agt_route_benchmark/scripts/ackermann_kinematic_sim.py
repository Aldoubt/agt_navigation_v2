#!/usr/bin/env python3
from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from agt_route_benchmark.ackermann_kinematics import AckermannState, step_ackermann_twist


class AckermannKinematicSim(Node):
    def __init__(self) -> None:
        super().__init__("agt_route_benchmark_ackermann_sim")
        self._wheel_base = float(self.declare_parameter("wheel_base_m", 0.60).value)
        self._min_radius = float(self.declare_parameter("min_turning_radius_m", 1.50).value)
        self._rate_hz = float(self.declare_parameter("update_rate_hz", 50.0).value)
        self._command_timeout_s = float(self.declare_parameter("command_timeout_s", 0.5).value)
        self._odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self._base_frame = str(self.declare_parameter("base_frame", "base_footprint").value)
        self._odom_topic = str(self.declare_parameter("odom_topic", "/agt/mapping/odometry").value)
        self._cmd_topic = str(self.declare_parameter("cmd_vel_topic", "/agt/navigation/cmd_vel_raw").value)
        self._state = AckermannState(
            float(self.declare_parameter("initial_x", 0.0).value),
            float(self.declare_parameter("initial_y", 0.0).value),
            float(self.declare_parameter("initial_yaw", 0.0).value),
        )
        if self._wheel_base <= 0.0 or self._min_radius <= 0.0 or self._rate_hz <= 0.0:
            raise ValueError("wheel_base_m, min_turning_radius_m and update_rate_hz must be positive")
        if self._command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive")

        self._linear = 0.0
        self._angular = 0.0
        self._last_command_s = float("-inf")
        self._last_update_s = self._now_s()
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 20)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Twist, self._cmd_topic, self._cmd_callback, 20)
        self.create_timer(1.0 / self._rate_hz, self._update)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_callback(self, message: Twist) -> None:
        self._linear = float(message.linear.x)
        self._angular = float(message.angular.z)
        self._last_command_s = self._now_s()

    def _update(self) -> None:
        now_s = self._now_s()
        dt = max(0.0, min(0.1, now_s - self._last_update_s))
        self._last_update_s = now_s
        if now_s - self._last_command_s > self._command_timeout_s:
            linear = 0.0
            angular = 0.0
        else:
            linear = self._linear
            angular = self._angular

        self._state, diagnostic = step_ackermann_twist(
            self._state,
            linear_velocity_mps=linear,
            angular_velocity_rps=angular,
            dt_s=dt,
            wheel_base_m=self._wheel_base,
            min_turning_radius_m=self._min_radius,
        )
        self._publish(linear, float(diagnostic["effective_yaw_rate_rps"]))

    def _publish(self, linear_velocity: float, yaw_rate: float) -> None:
        stamp = self.get_clock().now().to_msg()
        half = 0.5 * self._state.yaw_rad
        qz = math.sin(half)
        qw = math.cos(half)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._state.x_m
        odom.pose.pose.position.y = self._state.y_m
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(linear_velocity)
        odom.twist.twist.angular.z = float(yaw_rate)
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._state.x_m
        transform.transform.translation.y = self._state.y_m
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannKinematicSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
