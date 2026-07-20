#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from apply_swept_footprint_to_map import apply_swept_cells, read_p5, write_p5
from static_obstacle_evidence import (
    inside_or_near_polygon,
    interpolate_pose,
    rasterize_footprint_cells,
)


def fit_ground_plane(
    local_points,
    *,
    seed,
    distance_threshold=0.08,
    maximum_slope_degrees=20.0,
    minimum_candidates=50,
    maximum_samples=1500,
    iterations=32,
):
    """Fit z = ax + by + c using constrained deterministic RANSAC."""
    if local_points.shape[0] < minimum_candidates:
        return None
    points = np.asarray(local_points, dtype=np.float64)
    rng = np.random.default_rng(seed)
    if points.shape[0] > maximum_samples:
        points = points[rng.choice(points.shape[0], maximum_samples, replace=False)]
    best = None
    maximum_slope = math.tan(math.radians(maximum_slope_degrees))
    design = np.column_stack((points[:, 0], points[:, 1], np.ones(points.shape[0])))
    for _ in range(iterations):
        indexes = rng.choice(points.shape[0], 3, replace=False)
        try:
            coefficients = np.linalg.solve(design[indexes], points[indexes, 2])
        except np.linalg.LinAlgError:
            continue
        if math.hypot(coefficients[0], coefficients[1]) > maximum_slope:
            continue
        inliers = np.abs(points[:, 2] - design @ coefficients) <= distance_threshold
        count = int(np.count_nonzero(inliers))
        if best is None or count > best[0]:
            best = count, inliers
    if best is None or best[0] < minimum_candidates:
        return None
    coefficients = np.linalg.lstsq(
        design[best[1]], points[best[1], 2], rcond=None
    )[0]
    return coefficients, best[0] / points.shape[0]


def update_cell_statistics(statistics, cells, timestamp):
    for x, y in cells:
        key = int(x), int(y)
        value = statistics.get(key)
        if value is None:
            statistics[key] = [1, float(timestamp), float(timestamp)]
        else:
            value[0] += 1
            value[2] = float(timestamp)


def select_cells(statistics, *, minimum_observations, minimum_span):
    return {
        key
        for key, (count, first, last) in statistics.items()
        if count >= minimum_observations and last - first >= minimum_span
    }


def pad_cells(cells, padding_cells):
    if padding_cells <= 0:
        return set(cells)
    output = set()
    for x, y in cells:
        for dx in range(-padding_cells, padding_cells + 1):
            for dy in range(-padding_cells, padding_cells + 1):
                output.add((x + dx, y + dy))
    return output


def overlay_occupied(image, cells, *, origin_x, origin_y, resolution):
    output = image.copy()
    if not cells:
        return output
    height, width = output.shape
    array = np.asarray(tuple(cells), dtype=np.int64)
    world_x = (array[:, 0] + 0.5) * resolution
    world_y = (array[:, 1] + 0.5) * resolution
    columns = np.floor((world_x - origin_x) / resolution).astype(np.int64)
    map_rows = np.floor((world_y - origin_y) / resolution).astype(np.int64)
    rows = height - 1 - map_rows
    valid = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    output[rows[valid], columns[valid]] = 0
    return output


def load_odometry(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic = "/agt/mapping/odometry"
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in types:
        raise ValueError(f"bag does not contain {topic}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    samples = []
    while reader.has_next():
        _, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, get_message(types[topic]))
        stamp = message.header.stamp
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        samples.append(
            (
                stamp.sec + stamp.nanosec * 1e-9,
                position.x,
                position.y,
                position.z,
                yaw,
            )
        )
    return samples


def write_variant(output_directory, name, metadata, image):
    prefix = output_directory / name
    write_p5(prefix.with_suffix(".pgm"), image, float(metadata["resolution"]))
    output_metadata = dict(metadata)
    output_metadata["image"] = f"{name}.pgm"
    prefix.with_suffix(".yaml").write_text(
        yaml.safe_dump(output_metadata, sort_keys=False), encoding="utf-8"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Generate ground, temporal, and provisional height-layer map variants."
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--baseline-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--platform-profile", required=True)
    parser.add_argument("--minimum-observations", type=int, default=3)
    parser.add_argument("--minimum-static-span", type=float, default=0.5)
    parser.add_argument("--ground-distance", type=float, default=0.08)
    parser.add_argument("--minimum-obstacle-height", type=float, default=0.10)
    parser.add_argument("--low-layer-top", type=float, default=0.35)
    parser.add_argument("--provisional-clearance-height", type=float, default=0.65)
    parser.add_argument("--maximum-obstacle-height", type=float, default=2.0)
    parser.add_argument("--self-filter-padding", type=float, default=0.12)
    parser.add_argument("--obstacle-padding", type=float, default=0.05)
    parser.add_argument("--sweep-clearance", type=float, default=0.05)
    args = parser.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    bag_path = Path(args.bag).resolve()
    baseline_yaml = Path(args.baseline_yaml).resolve()
    output_directory = Path(args.output_dir).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    metadata = yaml.safe_load(baseline_yaml.read_text(encoding="utf-8"))
    baseline = read_p5((baseline_yaml.parent / metadata["image"]).resolve())
    resolution = float(metadata["resolution"])
    origin_x, origin_y = map(float, metadata["origin"][:2])
    profile = yaml.safe_load(Path(args.platform_profile).read_text(encoding="utf-8"))
    footprint = profile["platform"]["geometry"]["navigation_footprint"]
    polygon = np.asarray(footprint, dtype=np.float64)
    odometry = load_odometry(bag_path)
    swept_cells = set()
    for _, x, y, _, yaw in odometry:
        cells = rasterize_footprint_cells(
            base_x=x,
            base_y=y,
            base_yaw=yaw,
            footprint=footprint,
            padding=args.sweep_clearance,
            resolution=resolution,
        )
        swept_cells.update((int(cx), int(cy)) for cx, cy in cells)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic = "/agt/mapping/registered_points"
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in types:
        raise ValueError(f"bag does not contain {topic}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    all_statistics = {}
    low_statistics = {}
    body_statistics = {}
    collision_statistics = {}
    overhead_statistics = {}
    planes = []
    plane_failures = 0
    pose_mismatches = 0
    cloud_count = 0
    while reader.has_next():
        _, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, get_message(types[topic]))
        stamp = message.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        pose = interpolate_pose(odometry, timestamp, 0.25)
        cloud_count += 1
        if pose is None:
            pose_mismatches += 1
            continue
        points = point_cloud2.read_points_numpy(
            message, field_names=["x", "y", "z"], skip_nans=True
        ).astype(np.float64)
        if points.size == 0:
            continue
        base_x, base_y, base_z, base_yaw = pose
        dx = points[:, 0] - base_x
        dy = points[:, 1] - base_y
        relative_z = points[:, 2] - base_z
        cosine, sine = math.cos(base_yaw), math.sin(base_yaw)
        local_xy = np.column_stack(
            (cosine * dx + sine * dy, -sine * dx + cosine * dy)
        )
        finite = np.isfinite(points).all(axis=1)
        self_return = inside_or_near_polygon(
            local_xy, polygon, args.self_filter_padding
        )
        radius = np.hypot(dx, dy)
        ground_candidates = (
            finite
            & ~self_return
            & (radius >= 1.0)
            & (radius <= 20.0)
            & (relative_z >= -0.5)
            & (relative_z <= 0.5)
        )
        candidate_points = np.column_stack(
            (dx[ground_candidates], dy[ground_candidates], relative_z[ground_candidates])
        )
        fitted = fit_ground_plane(
            candidate_points,
            seed=cloud_count,
            distance_threshold=args.ground_distance,
        )
        if fitted is None:
            plane_failures += 1
            coefficients = np.asarray([0.0, 0.0, -0.044])
            ratio = 0.0
        else:
            coefficients, ratio = fitted
        planes.append((float(coefficients[0]), float(coefficients[1]), float(coefficients[2]), ratio))
        height = relative_z - (
            coefficients[0] * dx + coefficients[1] * dy + coefficients[2]
        )
        common = finite & ~self_return & (height >= args.minimum_obstacle_height)
        common &= height <= args.maximum_obstacle_height
        grid = np.floor(points[:, :2] / resolution).astype(np.int64)
        all_cells = np.unique(grid[common], axis=0)
        collision_cells = np.unique(
            grid[common & (height < args.provisional_clearance_height)], axis=0
        )
        low_cells = np.unique(
            grid[common & (height < args.low_layer_top)], axis=0
        )
        body_cells = np.unique(
            grid[
                common
                & (height >= args.low_layer_top)
                & (height < args.provisional_clearance_height)
            ],
            axis=0,
        )
        overhead_cells = np.unique(
            grid[common & (height >= args.provisional_clearance_height)], axis=0
        )
        update_cell_statistics(all_statistics, all_cells, timestamp)
        update_cell_statistics(low_statistics, low_cells, timestamp)
        update_cell_statistics(body_statistics, body_cells, timestamp)
        update_cell_statistics(collision_statistics, collision_cells, timestamp)
        update_cell_statistics(overhead_statistics, overhead_cells, timestamp)

    observed_all = select_cells(
        all_statistics,
        minimum_observations=args.minimum_observations,
        minimum_span=0.0,
    )
    static_all = select_cells(
        all_statistics,
        minimum_observations=args.minimum_observations,
        minimum_span=args.minimum_static_span,
    )
    static_collision = select_cells(
        collision_statistics,
        minimum_observations=args.minimum_observations,
        minimum_span=args.minimum_static_span,
    )
    padding_cells = int(math.ceil(args.obstacle_padding / resolution))
    variants = {
        "ground_only": pad_cells(observed_all, padding_cells),
        "ground_temporal": pad_cells(static_all, padding_cells),
        "ground_temporal_layered_provisional": pad_cells(
            static_collision, padding_cells
        ),
    }
    report = {
        "eligible_for_execution": False,
        "reason": "provisional vehicle clearance height is not physically verified",
        "clouds": cloud_count,
        "odometry_poses": len(odometry),
        "ground_plane_failures": plane_failures,
        "pose_mismatches": pose_mismatches,
        "swept_cells": len(swept_cells),
        "parameters": vars(args),
        "variants": {},
    }
    for name, occupied in variants.items():
        image = overlay_occupied(
            baseline,
            occupied,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
        )
        image, swept_changed = apply_swept_cells(
            image,
            swept_cells,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
        )
        write_variant(output_directory, name, metadata, image)
        report["variants"][name] = {
            "evidence_cells_with_padding": len(occupied),
            "occupied_pixels": int(np.count_nonzero(image == 0)),
            "sweep_changed_pixels": swept_changed,
        }
    plane_array = np.asarray(planes)
    static_low = select_cells(
        low_statistics,
        minimum_observations=args.minimum_observations,
        minimum_span=args.minimum_static_span,
    )
    static_body = select_cells(
        body_statistics,
        minimum_observations=args.minimum_observations,
        minimum_span=args.minimum_static_span,
    )
    static_overhead = select_cells(
        overhead_statistics,
        minimum_observations=args.minimum_observations,
        minimum_span=args.minimum_static_span,
    )
    report["height_layer_cells"] = {
        "low_0_10_to_0_35_m": len(static_low),
        "body_0_35_to_0_65_m": len(static_body),
        "overhead_0_65_to_2_00_m": len(static_overhead),
    }
    report["ground_model"] = {
        "successful_planes": len(planes) - plane_failures,
        "median_slope_degrees": float(
            np.degrees(np.arctan(np.median(np.hypot(plane_array[:, 0], plane_array[:, 1]))))
        ),
        "median_base_relative_ground_m": float(np.median(plane_array[:, 2])),
        "median_inlier_ratio": float(np.median(plane_array[:, 3])),
    }
    (output_directory / "comparison_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    print(json.dumps(report, sort_keys=True))


if __name__ == "__main__":
    main()
