#!/usr/bin/env python3
"""Verify an existing rosbag2 without changing it."""

import argparse
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify an existing rosbag2")
    parser.add_argument("bag")
    parser.add_argument("--report")
    args = parser.parse_args()
    command = [
        "ros2", "run", "rosbag_sensor_trimmer", "rosbag_sensor_trimmer_cli",
        "--input", args.bag, "--verify-only",
    ]
    if args.report:
        command.extend(["--report", args.report])
    return subprocess.call(command)


if __name__ == "__main__":
    sys.exit(main())
