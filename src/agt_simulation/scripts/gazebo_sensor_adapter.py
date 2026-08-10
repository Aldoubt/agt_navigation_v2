#!/usr/bin/env python3

"""Normalize Gazebo sensor topics and frame IDs for AGT software validation."""

from __future__ import annotations

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


class GazeboSensorAdapter(Node):
    def __init__(self) -> None:
        super().__init__("agt_gazebo_sensor_adapter")
        raw_scan = str(self.declare_parameter("raw_scan_topic", "/simulation/raw/scan").value)
        raw_imu = str(self.declare_parameter("raw_imu_topic", "/simulation/raw/imu").value)
        scan_topic = str(
            self.declare_parameter("scan_topic", "/agt/sensors/lidar/scan").value
        )
        imu_topic = str(
            self.declare_parameter("imu_topic", "/agt/sensors/imu/data").value
        )
        self.lidar_frame = str(self.declare_parameter("lidar_frame", "lidar_link").value)
        self.imu_frame = str(self.declare_parameter("imu_frame", "imu_link").value)

        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.imu_pub = self.create_publisher(Imu, imu_topic, 50)
        self.create_subscription(LaserScan, raw_scan, self._scan, 10)
        self.create_subscription(Imu, raw_imu, self._imu, 50)

    def _scan(self, message: LaserScan) -> None:
        output = copy.deepcopy(message)
        output.header.frame_id = self.lidar_frame
        self.scan_pub.publish(output)

    def _imu(self, message: Imu) -> None:
        output = copy.deepcopy(message)
        output.header.frame_id = self.imu_frame
        self.imu_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboSensorAdapter()
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
