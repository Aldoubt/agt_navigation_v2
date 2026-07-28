#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

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


def project_cells(cells, *, width, height, origin_x, origin_y, resolution):
    if not cells:
        empty = np.empty(0, dtype=np.int64)
        return empty, empty, np.zeros(0, dtype=bool)
    array = np.asarray(tuple(cells), dtype=np.int64)
    world_x = (array[:, 0] + 0.5) * resolution
    world_y = (array[:, 1] + 0.5) * resolution
    columns = np.floor((world_x - origin_x) / resolution).astype(np.int64)
    map_rows = np.floor((world_y - origin_y) / resolution).astype(np.int64)
    rows = height - 1 - map_rows
    valid = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    return rows, columns, valid


def overlay_occupied(image, cells, *, origin_x, origin_y, resolution):
    output = image.copy()
    if not cells:
        return output, 0, 0
    height, width = output.shape
    rows, columns, valid = project_cells(
        cells,
        width=width,
        height=height,
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
    )
    output[rows[valid], columns[valid]] = 0
    applied = int(np.count_nonzero(valid))
    return output, applied, len(cells) - applied


def _aligned_floor(value, resolution):
    return math.floor(value / resolution) * resolution


def _aligned_ceil(value, resolution):
    return math.ceil(value / resolution) * resolution


def expand_baseline(
    image,
    *,
    origin_x,
    origin_y,
    resolution,
    odometry,
    evidence_range,
    padding,
):
    """Expand a trinary baseline without inventing free space."""
    if not odometry:
        raise ValueError("ray-traced baseline rebuild requires mapping odometry")
    old_height, old_width = image.shape
    pose_x = [sample[1] for sample in odometry]
    pose_y = [sample[2] for sample in odometry]
    minimum_x = _aligned_floor(
        min(origin_x, min(pose_x) - evidence_range) - padding, resolution
    )
    minimum_y = _aligned_floor(
        min(origin_y, min(pose_y) - evidence_range) - padding, resolution
    )
    maximum_x = _aligned_ceil(
        max(origin_x + old_width * resolution, max(pose_x) + evidence_range) + padding,
        resolution,
    )
    maximum_y = _aligned_ceil(
        max(origin_y + old_height * resolution, max(pose_y) + evidence_range) + padding,
        resolution,
    )
    width = int(round((maximum_x - minimum_x) / resolution))
    height = int(round((maximum_y - minimum_y) / resolution))
    expanded = np.full((height, width), 205, dtype=np.uint8)
    column = int(round((origin_x - minimum_x) / resolution))
    old_map_row = int(round((origin_y - minimum_y) / resolution))
    row = height - old_map_row - old_height
    expanded[row : row + old_height, column : column + old_width] = image
    return expanded, minimum_x, minimum_y, {
        "source_width": old_width,
        "source_height": old_height,
        "source_origin": [float(origin_x), float(origin_y)],
        "width": width,
        "height": height,
        "origin": [float(minimum_x), float(minimum_y)],
        "metric_bounds": [
            float(minimum_x),
            float(minimum_y),
            float(maximum_x),
            float(maximum_y),
        ],
        "padding_m": float(padding),
        "evidence_range_m": float(evidence_range),
    }


def _contiguous_groups(indexes, maximum_gap_bins):
    if len(indexes) == 0:
        return []
    breaks = np.flatnonzero(np.diff(indexes) > maximum_gap_bins + 1) + 1
    return np.split(indexes, breaks)


def raytrace_free_scan(
    draw,
    points,
    *,
    pose,
    sensor_offset_xy,
    origin_x,
    origin_y,
    resolution,
    width,
    height,
    maximum_range,
    angular_resolution,
    minimum_relative_height,
    maximum_relative_height,
    maximum_gap_bins,
):
    """Rasterize deterministic 2D free-space rays from one registered cloud."""
    base_x, base_y, base_z, base_yaw = pose
    cosine, sine = math.cos(base_yaw), math.sin(base_yaw)
    sensor_x = base_x + cosine * sensor_offset_xy[0] - sine * sensor_offset_xy[1]
    sensor_y = base_y + sine * sensor_offset_xy[0] + cosine * sensor_offset_xy[1]
    dx = points[:, 0] - sensor_x
    dy = points[:, 1] - sensor_y
    distance = np.hypot(dx, dy)
    relative_z = points[:, 2] - base_z
    keep = np.isfinite(points).all(axis=1)
    keep &= distance >= resolution
    keep &= distance <= maximum_range
    keep &= relative_z >= minimum_relative_height
    keep &= relative_z <= maximum_relative_height
    if not np.any(keep):
        return {"rays": 0, "segments": 0, "points": 0}

    bin_count = int(math.ceil(2.0 * math.pi / angular_resolution))
    bin_width = 2.0 * math.pi / bin_count
    angles = np.arctan2(dy[keep], dx[keep])
    indexes = np.floor((angles + math.pi) / bin_width).astype(np.int64)
    indexes = np.clip(indexes, 0, bin_count - 1)
    ranges = np.zeros(bin_count, dtype=np.float64)
    np.maximum.at(ranges, indexes, distance[keep])
    valid_indexes = np.flatnonzero(ranges > 0.0)

    origin_column = int(math.floor((sensor_x - origin_x) / resolution))
    origin_map_row = int(math.floor((sensor_y - origin_y) / resolution))
    origin_row = height - 1 - origin_map_row
    origin_pixel = (origin_column, origin_row)
    segment_count = 0
    for group in _contiguous_groups(valid_indexes, maximum_gap_bins):
        group_angles = -math.pi + (group.astype(np.float64) + 0.5) * bin_width
        endpoint_x = sensor_x + np.cos(group_angles) * ranges[group]
        endpoint_y = sensor_y + np.sin(group_angles) * ranges[group]
        columns = np.floor((endpoint_x - origin_x) / resolution).astype(np.int64)
        map_rows = np.floor((endpoint_y - origin_y) / resolution).astype(np.int64)
        rows = height - 1 - map_rows
        endpoints = [(int(column), int(row)) for column, row in zip(columns, rows)]
        if len(endpoints) == 1:
            draw.line([origin_pixel, endpoints[0]], fill=254, width=1)
        else:
            draw.polygon([origin_pixel, *endpoints], fill=254)
        segment_count += 1
    return {
        "rays": int(len(valid_indexes)),
        "segments": segment_count,
        "points": int(np.count_nonzero(keep)),
    }


def raster_margins(image, value_test):
    coordinates = np.argwhere(value_test(image))
    if coordinates.size == 0:
        return None
    minimum_row, minimum_column = coordinates.min(axis=0)
    maximum_row, maximum_column = coordinates.max(axis=0)
    height, width = image.shape
    return {
        "left": int(minimum_column),
        "top": int(minimum_row),
        "right": int(width - 1 - maximum_column),
        "bottom": int(height - 1 - maximum_row),
    }


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


def _transform_matrix(transform):
    translation = transform.translation
    quaternion = transform.rotation
    q = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=np.float64
    )
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm <= 1e-12:
        raise ValueError("static TF contains an invalid quaternion")
    x, y, z, w = q / norm
    rotation = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    output = np.eye(4, dtype=np.float64)
    output[:3, :3] = rotation
    output[:3, 3] = [translation.x, translation.y, translation.z]
    return output


def load_static_sensor_offset(
    bag_path, *, base_frame="base_footprint", sensor_frame="lidar_link"
):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topic = "/tf_static"
    if topic not in types:
        raise ValueError(f"bag does not contain {topic}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    graph = {}
    while reader.has_next():
        _, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, get_message(types[topic]))
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            graph.setdefault(parent, []).append(
                (child, _transform_matrix(transform.transform))
            )
    pending = [(base_frame, np.eye(4, dtype=np.float64))]
    visited = set()
    while pending:
        frame, accumulated = pending.pop(0)
        if frame == sensor_frame:
            return accumulated[:3, 3]
        if frame in visited:
            continue
        visited.add(frame)
        for child, transform in graph.get(frame, []):
            pending.append((child, accumulated @ transform))
    raise ValueError(f"static TF has no path from {base_frame} to {sensor_frame}")


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
    parser.add_argument(
        "--rebuild-raytraced-baseline",
        action="store_true",
        help="Rebuild deterministic 2D free-space rays from registered clouds.",
    )
    parser.add_argument("--raytrace-interval", type=float, default=1.0)
    parser.add_argument("--raytrace-max-range", type=float, default=40.0)
    parser.add_argument("--raytrace-angular-resolution", type=float, default=0.0025)
    parser.add_argument("--raytrace-min-relative-height", type=float, default=-2.0)
    parser.add_argument("--raytrace-max-relative-height", type=float, default=3.0)
    parser.add_argument("--raytrace-maximum-gap-bins", type=int, default=2)
    parser.add_argument(
        "--maximum-evidence-range",
        type=float,
        default=0.0,
        help="Reject obstacle evidence beyond this recorded base-pose range; zero disables it.",
    )
    parser.add_argument(
        "--grid-padding",
        type=float,
        default=0.0,
        help="Explicit unknown-space padding around the rebuilt ray/evidence extent.",
    )
    args = parser.parse_args()

    positive_values = {
        "minimum_observations": args.minimum_observations,
        "minimum_static_span": args.minimum_static_span,
        "ground_distance": args.ground_distance,
        "maximum_obstacle_height": args.maximum_obstacle_height,
        "raytrace_interval": args.raytrace_interval,
        "raytrace_max_range": args.raytrace_max_range,
        "raytrace_angular_resolution": args.raytrace_angular_resolution,
    }
    invalid = [name for name, value in positive_values.items() if value <= 0]
    if invalid:
        raise ValueError(f"{invalid[0]} must be positive")
    if args.grid_padding < 0.0 or args.maximum_evidence_range < 0.0:
        raise ValueError("grid padding and maximum evidence range must be non-negative")
    if args.raytrace_max_relative_height <= args.raytrace_min_relative_height:
        raise ValueError("raytrace relative-height bounds are invalid")
    if args.raytrace_maximum_gap_bins < 0:
        raise ValueError("raytrace maximum gap bins must be non-negative")
    if args.rebuild_raytraced_baseline and args.maximum_evidence_range <= 0.0:
        raise ValueError("ray-traced baseline rebuild requires maximum_evidence_range")

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
    source_baseline = baseline
    canvas = {
        "source_width": int(baseline.shape[1]),
        "source_height": int(baseline.shape[0]),
        "source_origin": [origin_x, origin_y],
        "width": int(baseline.shape[1]),
        "height": int(baseline.shape[0]),
        "origin": [origin_x, origin_y],
        "metric_bounds": [
            origin_x,
            origin_y,
            origin_x + baseline.shape[1] * resolution,
            origin_y + baseline.shape[0] * resolution,
        ],
        "padding_m": 0.0,
        "evidence_range_m": 0.0,
    }
    if args.rebuild_raytraced_baseline:
        baseline, origin_x, origin_y, canvas = expand_baseline(
            baseline,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
            odometry=odometry,
            evidence_range=args.maximum_evidence_range,
            padding=args.grid_padding,
        )
        metadata = dict(metadata)
        metadata["origin"] = [origin_x, origin_y, float(metadata["origin"][2])]
    raytraced_image = Image.fromarray(baseline)
    raytraced_draw = ImageDraw.Draw(raytraced_image)
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
    empty_clouds = 0
    rejected_evidence_range_points = 0
    raytrace = {
        "enabled": bool(args.rebuild_raytraced_baseline),
        "selected_clouds": 0,
        "rays": 0,
        "segments": 0,
        "points": 0,
        "source_free_pixels": int(np.count_nonzero(source_baseline == 254)),
    }
    last_raytrace_timestamp = -math.inf
    sensor_offset = (
        load_static_sensor_offset(bag_path)
        if args.rebuild_raytraced_baseline
        else np.zeros(3, dtype=np.float64)
    )
    sensor_offset_xy = (float(sensor_offset[0]), float(sensor_offset[1]))
    raytrace["sensor_offset_base_footprint_m"] = [
        float(sensor_offset[0]),
        float(sensor_offset[1]),
        float(sensor_offset[2]),
    ]
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
            empty_clouds += 1
            continue
        base_x, base_y, base_z, base_yaw = pose
        if (
            args.rebuild_raytraced_baseline
            and timestamp - last_raytrace_timestamp >= args.raytrace_interval
        ):
            ray_statistics = raytrace_free_scan(
                raytraced_draw,
                points,
                pose=pose,
                sensor_offset_xy=sensor_offset_xy,
                origin_x=origin_x,
                origin_y=origin_y,
                resolution=resolution,
                width=baseline.shape[1],
                height=baseline.shape[0],
                maximum_range=args.raytrace_max_range,
                angular_resolution=args.raytrace_angular_resolution,
                minimum_relative_height=args.raytrace_min_relative_height,
                maximum_relative_height=args.raytrace_max_relative_height,
                maximum_gap_bins=args.raytrace_maximum_gap_bins,
            )
            raytrace["selected_clouds"] += 1
            for key in ("rays", "segments", "points"):
                raytrace[key] += ray_statistics[key]
            last_raytrace_timestamp = timestamp
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
        if args.maximum_evidence_range > 0.0:
            outside_evidence_range = common & (radius > args.maximum_evidence_range)
            rejected_evidence_range_points += int(
                np.count_nonzero(outside_evidence_range)
            )
            common &= radius <= args.maximum_evidence_range
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

    if args.rebuild_raytraced_baseline:
        baseline = np.asarray(raytraced_image, dtype=np.uint8).copy()
    raytrace["free_pixels"] = int(np.count_nonzero(baseline == 254))
    raytrace["new_free_pixels"] = max(
        0, raytrace["free_pixels"] - raytrace["source_free_pixels"]
    )

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
        "schema_version": 2,
        "eligible_for_execution": False,
        "reason": (
            "offline map production does not establish localization, planning, "
            "safety, or execution readiness"
        ),
        "clouds": cloud_count,
        "odometry_poses": len(odometry),
        "ground_plane_failures": plane_failures,
        "pose_mismatches": pose_mismatches,
        "empty_clouds": empty_clouds,
        "swept_cells": len(swept_cells),
        "rejected_evidence_range_points": rejected_evidence_range_points,
        "parameters": {**vars(args), "resolution": resolution},
        "canvas": canvas,
        "raytrace": raytrace,
        "variants": {},
    }
    for name, occupied in variants.items():
        image, applied_evidence, clipped_evidence = overlay_occupied(
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
        _, _, swept_valid = project_cells(
            swept_cells,
            width=image.shape[1],
            height=image.shape[0],
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
        )
        swept_applied = int(np.count_nonzero(swept_valid))
        report["variants"][name] = {
            "evidence_cells_with_padding": len(occupied),
            "evidence_cells_applied": applied_evidence,
            "evidence_cells_clipped": clipped_evidence,
            "occupied_pixels": int(np.count_nonzero(image == 0)),
            "sweep_changed_pixels": swept_changed,
            "swept_cells_applied": swept_applied,
            "swept_cells_clipped": len(swept_cells) - swept_applied,
            "known_edge_margin_cells": raster_margins(image, lambda data: data != 205),
            "occupied_edge_margin_cells": raster_margins(image, lambda data: data == 0),
            "eligible_for_candidate": False,
        }
        if name == "ground_temporal":
            minimum_edge_margin = max(
                1, int(math.floor(args.grid_padding / resolution)) - 1
            )
            margins = report["variants"][name]["known_edge_margin_cells"] or {}
            report["variants"][name]["eligible_for_candidate"] = bool(
                args.rebuild_raytraced_baseline
                and cloud_count > 0
                and plane_failures == 0
                and pose_mismatches == 0
                and raytrace["selected_clouds"] > 0
                and raytrace["rays"] > 0
                and clipped_evidence == 0
                and len(swept_cells) - swept_applied == 0
                and margins
                and min(margins.values()) >= minimum_edge_margin
            )
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
    report["selected_candidate"] = "ground_temporal"
    report["eligible_for_candidate"] = report["variants"]["ground_temporal"][
        "eligible_for_candidate"
    ]
    (output_directory / "comparison_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    print(json.dumps(report, sort_keys=True))


if __name__ == "__main__":
    main()
