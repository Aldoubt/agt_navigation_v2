from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Sequence

from .tracking_metrics import ExecutedPose

EXECUTED_TRAJECTORY_FIELDS = ("stamp_s", "x_m", "y_m", "yaw_rad")


def read_executed_trajectory_csv(path: Path | str) -> tuple[ExecutedPose, ...]:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != EXECUTED_TRAJECTORY_FIELDS:
            raise ValueError(f"executed trajectory CSV header must be {EXECUTED_TRAJECTORY_FIELDS}")
        poses = tuple(
            ExecutedPose(
                stamp_s=float(row["stamp_s"]),
                x_m=float(row["x_m"]),
                y_m=float(row["y_m"]),
                yaw_rad=float(row["yaw_rad"]),
            )
            for row in reader
        )
    if not poses:
        raise ValueError("executed trajectory CSV must contain at least one pose")
    previous = None
    for pose in poses:
        values = (pose.stamp_s, pose.x_m, pose.y_m, pose.yaw_rad)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("executed trajectory CSV contains non-finite values")
        if previous is not None and pose.stamp_s < previous:
            raise ValueError("executed trajectory timestamps must be monotonic")
        previous = pose.stamp_s
    return poses


def write_executed_trajectory_csv(poses: Sequence[ExecutedPose], path: Path | str) -> Path:
    if not poses:
        raise ValueError("executed trajectory requires at least one pose")
    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    previous = None
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=EXECUTED_TRAJECTORY_FIELDS)
        writer.writeheader()
        for pose in poses:
            values = (pose.stamp_s, pose.x_m, pose.y_m, pose.yaw_rad)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("executed trajectory contains non-finite values")
            if previous is not None and pose.stamp_s < previous:
                raise ValueError("executed trajectory timestamps must be monotonic")
            writer.writerow(
                {
                    "stamp_s": f"{pose.stamp_s:.9f}",
                    "x_m": f"{pose.x_m:.9f}",
                    "y_m": f"{pose.y_m:.9f}",
                    "yaw_rad": f"{pose.yaw_rad:.9f}",
                }
            )
            previous = pose.stamp_s
    return output
