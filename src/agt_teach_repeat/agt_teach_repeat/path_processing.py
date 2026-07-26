"""Deterministic teach-path filtering, resampling, and metrics geometry."""

import bisect
import math

from .path_types import (
    PathPose,
    ProcessedPath,
    ProcessingConfig,
    ProcessingReport,
    TeachRepeatError,
    TransformSE2,
    quaternion_from_yaw,
    wrap_angle,
)


EPSILON = 1.0e-9


def path_length(poses):
    return sum(
        math.hypot(second.x - first.x, second.y - first.y)
        for first, second in zip(poses, poses[1:])
    )


def _cumulative_lengths(poses):
    output = [0.0]
    for first, second in zip(poses, poses[1:]):
        output.append(output[-1] + math.hypot(second.x - first.x, second.y - first.y))
    return output


def _interpolate_pose(first, second, ratio, timestamp_ns):
    yaw_delta = wrap_angle(second.yaw - first.yaw)
    yaw = wrap_angle(first.yaw + ratio * yaw_delta)
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    return PathPose(
        timestamp_ns=int(timestamp_ns),
        x=first.x + ratio * (second.x - first.x),
        y=first.y + ratio * (second.y - first.y),
        z=first.z + ratio * (second.z - first.z),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        linear_x=first.linear_x + ratio * (second.linear_x - first.linear_x),
        linear_y=first.linear_y + ratio * (second.linear_y - first.linear_y),
        angular_z=first.angular_z + ratio * (second.angular_z - first.angular_z),
        frame_id=first.frame_id,
        child_frame_id=first.child_frame_id,
    )


def resample_by_arclength(poses, spacing, maximum_point_count=20000):
    poses = tuple(poses)
    if len(poses) < 2:
        raise TeachRepeatError("path_too_short", "path requires at least two poses")
    if not math.isfinite(float(spacing)) or spacing <= 0.0:
        raise TeachRepeatError(
            "invalid_resample_distance", "resample distance must be positive"
        )
    cumulative = _cumulative_lengths(poses)
    total = cumulative[-1]
    if total <= EPSILON:
        raise TeachRepeatError("zero_length_path", "path must contain translation")
    count = int(math.floor(total / spacing))
    targets = [index * spacing for index in range(count + 1)]
    if total - targets[-1] > EPSILON:
        targets.append(total)
    else:
        targets[-1] = total
    if len(targets) > maximum_point_count:
        raise TeachRepeatError(
            "maximum_point_count_exceeded", "resampled path exceeds point limit"
        )
    output = []
    for target in targets:
        right = bisect.bisect_right(cumulative, target)
        index = min(max(right - 1, 0), len(poses) - 2)
        segment = cumulative[index + 1] - cumulative[index]
        ratio = 0.0 if segment <= EPSILON else (target - cumulative[index]) / segment
        stamp = round(
            poses[index].timestamp_ns
            + ratio * (poses[index + 1].timestamp_ns - poses[index].timestamp_ns)
        )
        output.append(
            _interpolate_pose(poses[index], poses[index + 1], ratio, stamp)
        )
    output[0] = poses[0]
    output[-1] = poses[-1]
    return tuple(output)


def smooth_positions(poses, window, maximum_deviation):
    poses = tuple(poses)
    if len(poses) <= 2 or window <= 1 or maximum_deviation <= 0.0:
        return poses, 0.0
    radius = window // 2
    output = [poses[0]]
    observed_maximum = 0.0
    for index in range(1, len(poses) - 1):
        if index < radius or index + radius >= len(poses):
            output.append(poses[index])
            continue
        start = index - radius
        end = index + radius + 1
        target_x = sum(item.x for item in poses[start:end]) / (end - start)
        target_y = sum(item.y for item in poses[start:end]) / (end - start)
        delta_x = target_x - poses[index].x
        delta_y = target_y - poses[index].y
        deviation = math.hypot(delta_x, delta_y)
        if deviation > maximum_deviation:
            scale = maximum_deviation / deviation
            delta_x *= scale
            delta_y *= scale
            deviation = maximum_deviation
        observed_maximum = max(observed_maximum, deviation)
        pose = poses[index]
        output.append(
            PathPose(
                **{
                    **pose.__dict__,
                    "x": pose.x + delta_x,
                    "y": pose.y + delta_y,
                }
            )
        )
    output.append(poses[-1])
    return tuple(output), observed_maximum


def _rebuild_yaw(poses, minimum_translation):
    output = []
    for index, pose in enumerate(poses):
        speed = math.hypot(pose.linear_x, pose.linear_y)
        if speed <= minimum_translation:
            yaw = pose.yaw
        else:
            before = poses[max(0, index - 1)]
            after = poses[min(len(poses) - 1, index + 1)]
            delta_x = after.x - before.x
            delta_y = after.y - before.y
            yaw = (
                pose.yaw
                if math.hypot(delta_x, delta_y) <= EPSILON
                else math.atan2(delta_y, delta_x)
            )
        output.append(pose.with_yaw(yaw, frame_id="map"))
    return tuple(output)


def _inject_in_place_rotations(processed, source, config):
    source_lengths = _cumulative_lengths(source)
    processed_lengths = _cumulative_lengths(processed)
    candidates = [
        (distance, 1, index, pose)
        for index, (distance, pose) in enumerate(zip(processed_lengths, processed))
    ]
    sequence = len(candidates)
    for index, (first, second) in enumerate(zip(source, source[1:])):
        translation = math.hypot(second.x - first.x, second.y - first.y)
        yaw_change = abs(wrap_angle(second.yaw - first.yaw))
        if (
            translation <= config.minimum_translation_m
            and yaw_change >= config.minimum_yaw_change_rad
        ):
            candidates.append((source_lengths[index], 0, sequence, first))
            sequence += 1
            candidates.append((source_lengths[index + 1], 2, sequence, second))
            sequence += 1
    candidates.sort(key=lambda item: (item[0], item[1], item[2]))
    output = []
    for _distance, _rank, _index, pose in candidates:
        if output:
            previous = output[-1]
            if (
                math.hypot(pose.x - previous.x, pose.y - previous.y) <= EPSILON
                and abs(wrap_angle(pose.yaw - previous.yaw)) <= EPSILON
            ):
                continue
        output.append(pose.with_yaw(pose.yaw, frame_id="map"))
    if len(output) > config.maximum_point_count:
        raise TeachRepeatError(
            "maximum_point_count_exceeded",
            "path with in-place rotations exceeds point limit",
        )
    return tuple(output)


def path_curvatures(poses):
    if len(poses) < 2:
        return tuple()
    values = [0.0]
    for first, second in zip(poses, poses[1:]):
        distance = math.hypot(second.x - first.x, second.y - first.y)
        yaw_change = abs(wrap_angle(second.yaw - first.yaw))
        if distance > EPSILON:
            values.append(yaw_change / distance)
        else:
            values.append(math.inf if yaw_change > EPSILON else 0.0)
    return tuple(values)


def _deduplicate_and_normalize(raw_poses, config, warnings):
    normalized = []
    seen_timestamps = set()
    removed_non_finite = 0
    for input_index, item in enumerate(raw_poses):
        try:
            pose = item.normalized()
        except TeachRepeatError as exc:
            if exc.code in {"non_finite_pose", "non_finite_quaternion"}:
                removed_non_finite += 1
                continue
            raise
        if pose.timestamp_ns in seen_timestamps:
            continue
        if normalized and pose.timestamp_ns < normalized[-1].timestamp_ns:
            raise TeachRepeatError(
                "non_monotonic_time", "pose timestamps must be monotonic"
            )
        seen_timestamps.add(pose.timestamp_ns)
        if normalized:
            translation = math.hypot(
                pose.x - normalized[-1].x, pose.y - normalized[-1].y
            )
            yaw_change = abs(wrap_angle(pose.yaw - normalized[-1].yaw))
            if (
                translation < config.minimum_translation_m
                and yaw_change < config.minimum_yaw_change_rad
            ):
                is_last = input_index == len(raw_poses) - 1
                if not is_last or (translation <= EPSILON and yaw_change <= EPSILON):
                    continue
        normalized.append(pose)
    if removed_non_finite:
        warnings.append(f"removed {removed_non_finite} non-finite poses")
    return tuple(normalized)


def extract_control_points(poses, config):
    curvatures = path_curvatures(poses)
    cumulative = _cumulative_lengths(poses)
    selected = {0, len(poses) - 1}
    last_distance = 0.0
    for index in range(1, len(poses) - 1):
        yaw_before = abs(wrap_angle(poses[index].yaw - poses[index - 1].yaw))
        yaw_after = abs(wrap_angle(poses[index + 1].yaw - poses[index].yaw))
        if (
            curvatures[index] >= config.control_point_curvature_threshold
            or yaw_before >= 0.25
            or yaw_after >= 0.25
            or cumulative[index] - last_distance >= config.control_point_spacing_m
        ):
            selected.add(index)
            last_distance = cumulative[index]
    ordered = sorted(selected)
    if len(ordered) > config.maximum_control_points:
        interior_slots = config.maximum_control_points - 2
        interior = ordered[1:-1]
        chosen = [
            interior[
                round(
                    index
                    * (len(interior) - 1)
                    / max(interior_slots - 1, 1)
                )
            ]
            for index in range(interior_slots)
        ]
        ordered = [0, *sorted(set(chosen)), len(poses) - 1]
    return tuple(
        {
            "name": f"P{output_index:03d}",
            "x": float(f"{poses[index].x:.12g}"),
            "y": float(f"{poses[index].y:.12g}"),
            "theta": float(f"{poses[index].yaw:.12g}"),
        }
        for output_index, index in enumerate(ordered)
    )


def process_path(raw_poses, config=None, map_transform=None):
    config = config or ProcessingConfig()
    map_transform = map_transform or TransformSE2()
    raw_poses = tuple(raw_poses)
    if len(raw_poses) < 2:
        raise TeachRepeatError("path_too_short", "raw path requires at least two poses")
    warnings = []
    valid = _deduplicate_and_normalize(raw_poses, config, warnings)
    if len(valid) < 2:
        raise TeachRepeatError("path_too_short", "path has fewer than two valid poses")
    transformed = tuple(map_transform.apply(pose) for pose in valid)
    raw_length = path_length(transformed)
    if raw_length <= EPSILON:
        raise TeachRepeatError("zero_length_path", "path must contain translation")
    resampled = resample_by_arclength(
        transformed, config.resample_distance_m, config.maximum_point_count
    )
    if config.smoothing_enabled:
        smoothed, maximum_deviation = smooth_positions(
            resampled, config.smoothing_window, config.max_smoothing_deviation_m
        )
    else:
        smoothed, maximum_deviation = resampled, 0.0
    processed = _rebuild_yaw(smoothed, config.minimum_translation_m)
    processed = _inject_in_place_rotations(processed, transformed, config)
    curvatures = path_curvatures(processed)
    finite_curvatures = [value for value in curvatures if math.isfinite(value)]
    report = ProcessingReport(
        raw_count=len(raw_poses),
        valid_count=len(valid),
        processed_count=len(processed),
        raw_length_m=raw_length,
        processed_length_m=path_length(processed),
        maximum_smoothing_deviation_m=maximum_deviation,
        maximum_curvature=max(finite_curvatures, default=0.0),
        warnings=warnings,
    )
    return ProcessedPath(
        poses=processed,
        control_points=extract_control_points(processed, config),
        report=report,
    )


def nearest_segment_metrics(reference, x, y, yaw):
    if len(reference) < 2:
        raise TeachRepeatError("path_too_short", "reference path requires at least two poses")
    if not all(math.isfinite(float(value)) for value in (x, y, yaw)):
        raise TeachRepeatError("non_finite_pose", "evaluation pose must be finite")
    cumulative = _cumulative_lengths(reference)
    best = None
    for index, (first, second) in enumerate(zip(reference, reference[1:])):
        dx = second.x - first.x
        dy = second.y - first.y
        length_squared = dx * dx + dy * dy
        if length_squared <= EPSILON:
            ratio = 0.0
        else:
            ratio = max(0.0, min(1.0, ((x - first.x) * dx + (y - first.y) * dy) / length_squared))
        nearest_x = first.x + ratio * dx
        nearest_y = first.y + ratio * dy
        error_x = x - nearest_x
        error_y = y - nearest_y
        distance = math.hypot(error_x, error_y)
        segment_yaw = math.atan2(dy, dx) if length_squared > EPSILON else first.yaw
        signed_error = (-math.sin(segment_yaw) * error_x + math.cos(segment_yaw) * error_y)
        progress = cumulative[index] + ratio * math.sqrt(length_squared)
        candidate = (distance, index, signed_error, progress, segment_yaw)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
    total = cumulative[-1]
    return {
        "cross_track_error": best[2],
        "along_track_progress": best[3],
        "heading_error": wrap_angle(yaw - best[4]),
        "distance_to_goal": math.hypot(x - reference[-1].x, y - reference[-1].y),
        "completion_ratio": min(1.0, best[3] / total) if total > EPSILON else 0.0,
        "segment_index": best[1],
    }


def deterministic_percentile(values, percentile):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise TeachRepeatError("empty_samples", "percentile requires at least one value")
    if not all(math.isfinite(value) for value in ordered):
        raise TeachRepeatError("non_finite_sample", "percentile samples must be finite")
    percentile = float(percentile)
    if not 0.0 <= percentile <= 100.0:
        raise TeachRepeatError("invalid_percentile", "percentile must be in [0, 100]")
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction
