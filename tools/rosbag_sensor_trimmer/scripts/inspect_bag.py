#!/usr/bin/env python3
"""Small convenience wrapper for bag inspection."""

import argparse
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect a rosbag2 with rosbag_sensor_trimmer")
    parser.add_argument("bag")
    args = parser.parse_args()
    return subprocess.call([
        "ros2", "run", "rosbag_sensor_trimmer", "rosbag_sensor_trimmer_cli",
        "--input", args.bag, "--info",
    ])


if __name__ == "__main__":
    sys.exit(main())
