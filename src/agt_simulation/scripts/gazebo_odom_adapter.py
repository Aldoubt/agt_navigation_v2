#!/usr/bin/env python3

"""Adapt Gazebo ground-truth odometry to the AGT mapping odometry contract.

This node is simulation-only. It emulates the mapping/odometry provider by
publishing /agt/mapping/odometry and owning odom -> base_footprint while the
Gazebo validation launch is active. It never publishes map -> odom.
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GazeboOdomAdapter(Node):
    def __init__(self) -> None:
        super().__init__("agt_gazebo_odom_adapter")
        self.input_topic = str(
            self.declare_parameter("input_topic", "/simulation/bunker/odometry").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/agt/mapping/odometry").value
        )
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.base_frame = str(
            self.declare_parameter("base_frame", "base_footprint").value
        )
        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.input_topic, self._on_odom, 20)

    def _on_odom(self, message: Odometry) -> None:
        canonical = Odometry()
        canonical.header = message.header
        canonical.header.frame_id = self.odom_frame
        canonical.child_frame_id = self.base_frame
        canonical.pose = message.pose
        canonical.twist = message.twist
        self.publisher.publish(canonical)

        transform = TransformStamped()
        transform.header = canonical.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = canonical.pose.pose.position.x
        transform.transform.translation.y = canonical.pose.pose.position.y
        transform.transform.translation.z = canonical.pose.pose.position.z
        transform.transform.rotation = canonical.pose.pose.orientation
        self.tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboOdomAdapter()
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
