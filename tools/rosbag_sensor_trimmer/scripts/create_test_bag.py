#!/usr/bin/env python3
"""Create a tiny deterministic rosbag2 fixture for local integration tests."""

import argparse
import os
import shutil

import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


BASE_NS = 1_700_000_000_000_000_000


def make_time(timestamp_ns: int) -> Time:
    message_time = Time()
    message_time.sec = timestamp_ns // 1_000_000_000
    message_time.nanosec = timestamp_ns % 1_000_000_000
    return message_time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    args = parser.parse_args()
    if os.path.exists(args.output):
        shutil.rmtree(args.output)

    storage = rosbag2_py.StorageOptions(uri=args.output, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage, converter)
    topics = [
        ("/lidar", "sensor_msgs/msg/PointCloud2"),
        ("/imu", "sensor_msgs/msg/Imu"),
        ("/tf_static", "tf2_msgs/msg/TFMessage"),
        ("/unrelated", "std_msgs/msg/String"),
    ]
    for name, type_name in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=type_name, serialization_format="cdr", offered_qos_profiles=""))

    point_cloud = PointCloud2()
    imu = Imu()
    tf_message = TFMessage()
    unrelated = String(data="fixture")
    records = []
    for second in range(10):
        timestamp = BASE_NS + second * 1_000_000_000
        point_cloud.header.stamp = make_time(timestamp)
        records.append((timestamp, "/lidar", serialize_message(point_cloud)))
        unrelated.data = "fixture-" + str(second)
        records.append((timestamp, "/unrelated", serialize_message(unrelated)))
        if second == 0:
            records.append((timestamp, "/tf_static", serialize_message(tf_message)))
    for half_second in range(20):
        timestamp = BASE_NS + half_second * 500_000_000
        imu.header.stamp = make_time(timestamp)
        imu.angular_velocity.x = float(half_second)
        records.append((timestamp, "/imu", serialize_message(imu)))

    for timestamp, topic, data in sorted(records, key=lambda item: (item[0], item[1])):
        writer.write(topic, data, timestamp)


if __name__ == "__main__":
    main()
