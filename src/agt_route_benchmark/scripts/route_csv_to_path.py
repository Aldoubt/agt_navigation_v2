#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path

from agt_route_benchmark.path_io import read_path_csv


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish an exported benchmark CSV as nav_msgs/Path preview")
    parser.add_argument("--path-csv", required=True, type=Path)
    parser.add_argument("--topic", default="/agt/benchmark/path_preview")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()

    if args.frame_id != "map":
        raise SystemExit("Paper I route assets are map-frame only")
    points = read_path_csv(args.path_csv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Path as NavPath
    from geometry_msgs.msg import PoseStamped

    rclpy.init()
    node = Node("agt_route_csv_preview")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    pub = node.create_publisher(NavPath, args.topic, qos)

    msg = NavPath()
    msg.header.frame_id = args.frame_id
    for p in points:
        pose = PoseStamped()
        pose.header.frame_id = args.frame_id
        pose.pose.position.x = p.x_m
        pose.pose.position.y = p.y_m
        pose.pose.orientation.z = math.sin(p.yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(p.yaw_rad / 2.0)
        msg.poses.append(pose)

    period = 1.0 / max(args.rate, 0.1)

    def publish():
        now = node.get_clock().now().to_msg()
        msg.header.stamp = now
        for pose in msg.poses:
            pose.header.stamp = now
        pub.publish(msg)

    node.create_timer(period, publish)
    publish()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
