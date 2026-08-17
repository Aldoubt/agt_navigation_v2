#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from agt_route_benchmark.path_io import read_path_csv
from agt_route_benchmark.tracking_io import write_executed_trajectory_csv
from agt_route_benchmark.tracking_metrics import ExecutedPose, compute_tracking_metrics


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class TfTrajectoryRecorder(Node):
    def __init__(
        self,
        *,
        reference_csv: Path,
        executed_csv: Path,
        metrics_json: Path,
        fixed_frame: str,
        base_frame: str,
        rate_hz: float,
    ) -> None:
        super().__init__("agt_route_benchmark_tf_recorder")
        self.reference_csv = reference_csv.expanduser().resolve()
        self.executed_csv = executed_csv.expanduser().resolve()
        self.metrics_json = metrics_json.expanduser().resolve()
        self.fixed_frame = str(fixed_frame)
        self.base_frame = str(base_frame)
        self.rate_hz = float(rate_hz)
        if self.rate_hz <= 0.0 or not math.isfinite(self.rate_hz):
            raise ValueError("rate_hz must be positive and finite")
        if not self.reference_csv.is_file():
            raise ValueError(f"reference path.csv does not exist: {self.reference_csv}")
        self.reference = read_path_csv(self.reference_csv)
        self.samples: list[ExecutedPose] = []
        self._buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._listener = TransformListener(self._buffer, self, spin_thread=False)
        self._last_warn_ns = 0
        self.create_timer(1.0 / self.rate_hz, self._sample)

    def _sample(self) -> None:
        try:
            transform = self._buffer.lookup_transform(
                self.fixed_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=min(0.05, 0.5 / self.rate_hz)),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_warn_ns >= 2_000_000_000:
                self.get_logger().warning(
                    f"waiting for TF {self.fixed_frame}->{self.base_frame}: {exc}"
                )
                self._last_warn_ns = now_ns
            return

        stamp = transform.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = ExecutedPose(
            stamp_s=stamp_s,
            x_m=float(translation.x),
            y_m=float(translation.y),
            yaw_rad=_yaw_from_quaternion(rotation),
        )
        if self.samples and pose.stamp_s < self.samples[-1].stamp_s:
            self.get_logger().warning("TF timestamp moved backwards; sample discarded")
            return
        if self.samples:
            previous = self.samples[-1]
            if (
                pose.stamp_s == previous.stamp_s
                and pose.x_m == previous.x_m
                and pose.y_m == previous.y_m
                and pose.yaw_rad == previous.yaw_rad
            ):
                return
        self.samples.append(pose)

    def finalize(self) -> dict:
        if not self.samples:
            raise RuntimeError(
                f"no valid TF samples recorded for {self.fixed_frame}->{self.base_frame}"
            )
        write_executed_trajectory_csv(self.samples, self.executed_csv)
        metrics = compute_tracking_metrics(self.reference, self.samples)
        report = {
            "schema_version": "1.0",
            "trial_kind": "real_or_external_tf_tracking",
            "reference_geometry_modified": False,
            "fixed_frame": self.fixed_frame,
            "base_frame": self.base_frame,
            "reference_path_csv": str(self.reference_csv),
            "executed_trajectory_csv": str(self.executed_csv),
            **metrics,
        }
        self.metrics_json.parent.mkdir(parents=True, exist_ok=True)
        self.metrics_json.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        return report


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Record map->base TF during an RPP trial and score it against Paper I path.csv"
    )
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--executed", required=True, type=Path)
    parser.add_argument("--metrics", required=True, type=Path)
    parser.add_argument("--fixed-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--rate", type=float, default=20.0)
    known, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = TfTrajectoryRecorder(
        reference_csv=known.reference,
        executed_csv=known.executed,
        metrics_json=known.metrics,
        fixed_frame=known.fixed_frame,
        base_frame=known.base_frame,
        rate_hz=known.rate,
    )
    exit_code = 0
    try:
        node.get_logger().info(
            f"recording {known.fixed_frame}->{known.base_frame}; stop with Ctrl-C after the route trial"
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            report = node.finalize()
            print(json.dumps(report, indent=2, sort_keys=True))
        except Exception as exc:
            print(json.dumps({"success": False, "error": str(exc)}, indent=2), flush=True)
            exit_code = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
