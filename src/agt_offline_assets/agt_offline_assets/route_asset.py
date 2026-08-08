"""Semantic-route derivation and versioned CSV/manifest I/O."""

from dataclasses import dataclass, replace
import csv
import math
import os
from pathlib import Path
import shutil
from typing import Iterable

import yaml

from agt_ui_bridge.coverage_preview import (
    CoveragePreviewError,
    derive_inter_row_aisles,
    explicit_access_lane_swaths,
)
from agt_ui_bridge.platform_profile import load_platform_profile
from agt_ui_bridge.semantic_io import load_semantic_task

from .contracts import AssetContractError, RoutePolicy, load_yaml_mapping, sha256_file


_ROUTE_FIELDS = (
    "seq", "segment_id", "x", "y", "yaw", "direction", "v_ref", "curvature",
    "clearance", "semantic_ref", "event_ref",
)


@dataclass(frozen=True)
class RouteSample:
    seq: int
    segment_id: str
    x: float
    y: float
    yaw: float
    direction: str = "F"
    v_ref: float = 0.3
    curvature: float = 0.0
    clearance: float = -1.0
    semantic_ref: str = ""
    event_ref: str = ""


def derive_route_candidate(
    semantic_path: str | Path,
    policy_path: str | Path,
    platform_profile_path: str | Path,
    *,
    coverage_path: str | Path | None = None,
    default_speed_mps: float = 0.3,
) -> list[RouteSample]:
    """Derive a deterministic boustrophedon candidate from annotated semantic lanes.

    This MVP deliberately keeps inter-lane connectors straight. Full kinematic
    connector planning is a backend extension; the feasibility gate will reject a
    straight connector that violates footprint or minimum-turning-radius constraints.
    """
    policy = RoutePolicy.from_file(policy_path)
    if policy.planning_mode != "annotated_rows":
        raise AssetContractError(
            "route_policy_mode_unsupported",
            "semantic route MVP currently requires planning_mode=annotated_rows",
        )
    task = load_semantic_task(semantic_path, coverage_path)
    if task.read_only:
        raise AssetContractError(
            "semantic_task_read_only",
            "semantic task has integrity errors and cannot derive a route",
        )
    platform = load_platform_profile(platform_profile_path)
    if str(platform["name"]) != str(task.coverage.robot_profile):
        raise AssetContractError(
            "route_platform_mismatch",
            "semantic coverage robot_profile differs from selected platform profile",
        )
    speed = float(default_speed_mps)
    if not math.isfinite(speed) or speed <= 0.0:
        raise AssetContractError("route_speed_invalid", "default speed must be positive")

    semantic_map = task.semantic_map
    if policy.row_interpretation == "crop_centerlines":
        try:
            route_map = derive_inter_row_aisles(semantic_map)
        except CoveragePreviewError as exc:
            raise AssetContractError("route_aisle_derivation_failed", str(exc)) from exc
        features = [
            feature for feature in route_map.features
            if feature.enabled and feature.feature_type == "row_centerline"
        ]
        if not policy.use_access_lanes:
            features = [feature for feature in features if feature.id.startswith("preview_aisle_")]
    else:
        features = [
            feature for feature in semantic_map.features
            if feature.enabled and feature.feature_type == "row_centerline"
        ]
        if policy.use_access_lanes:
            features += explicit_access_lane_swaths(semantic_map)

    if not features:
        raise AssetContractError("route_no_lanes", "no enabled row/access lane is available")

    direction = _work_direction(semantic_map)
    normal = (-direction[1], direction[0])
    prepared = []
    for feature in features:
        points = [[float(value) for value in point[:2]] for point in feature.coordinates]
        if len(points) < 2:
            raise AssetContractError("route_lane_too_short", f"{feature.id} needs at least two points")
        if _dot((points[-1][0] - points[0][0], points[-1][1] - points[0][1]), direction) < 0.0:
            points.reverse()
        centroid = (
            sum(point[0] for point in points) / len(points),
            sum(point[1] for point in points) / len(points),
        )
        prepared.append((_dot(centroid, normal), str(feature.id), points))
    prepared.sort(key=lambda item: (-item[0], item[1]))

    samples: list[RouteSample] = []
    segment_number = 0
    previous_end = None
    for index, (_, feature_id, points) in enumerate(prepared):
        if index % 2 == 1:
            points = list(reversed(points))
        if previous_end is not None and math.dist(previous_end, points[0]) > 1e-6:
            connector_id = f"connector_{segment_number:03d}"
            connector = _resample_polyline(
                [previous_end, points[0]], policy.path_resolution_m
            )
            samples.extend(
                _samples_from_points(
                    connector,
                    connector_id,
                    "F",
                    speed,
                    "<connector>",
                    start_seq=len(samples),
                )
            )
            segment_number += 1
        segment_id = f"lane_{segment_number:03d}"
        route_points = _resample_polyline(points, policy.path_resolution_m)
        samples.extend(
            _samples_from_points(
                route_points,
                segment_id,
                "F",
                speed,
                feature_id,
                start_seq=len(samples),
            )
        )
        segment_number += 1
        previous_end = points[-1]

    if len(samples) < 2:
        raise AssetContractError("route_too_short", "derived route has fewer than two samples")
    return _recompute_curvature(samples)


def create_route_candidate_asset(
    *,
    map_manifest_path: str | Path,
    semantic_path: str | Path,
    policy_path: str | Path,
    platform_profile_path: str | Path,
    route_id: str,
    revision: int,
    coverage_path: str | Path | None = None,
    default_speed_mps: float = 0.3,
) -> Path:
    """Create a DRAFT Route Asset under the bound map version."""
    map_manifest_path = Path(map_manifest_path).expanduser().resolve()
    map_manifest = load_yaml_mapping(map_manifest_path)
    if str(map_manifest.get("state", "")).upper() != "READY":
        raise AssetContractError("route_map_not_ready", "Route Asset requires a READY map version")
    route_id = str(route_id).strip()
    if not route_id or int(revision) <= 0:
        raise AssetContractError("route_identity_invalid", "route_id and positive revision are required")
    semantic_path = Path(semantic_path).expanduser().resolve()
    policy_path = Path(policy_path).expanduser().resolve()
    platform_path = Path(platform_profile_path).expanduser().resolve()
    platform = load_platform_profile(platform_path)
    expected_platform_hash = str(map_manifest.get("platform_profile_sha256", ""))
    actual_platform_hash = sha256_file(platform_path)
    if expected_platform_hash and actual_platform_hash != expected_platform_hash:
        raise AssetContractError(
            "route_map_platform_mismatch",
            "platform profile hash differs from the bound map version",
        )

    samples = derive_route_candidate(
        semantic_path,
        policy_path,
        platform_path,
        coverage_path=coverage_path,
        default_speed_mps=default_speed_mps,
    )
    route_dir = map_manifest_path.parent / "routes" / route_id / str(int(revision))
    if route_dir.exists():
        raise AssetContractError("route_revision_exists", f"route revision already exists: {route_dir}")
    route_dir.mkdir(parents=True)
    policy_copy = route_dir / "policy.yaml"
    shutil.copy2(policy_path, policy_copy)
    route_csv = route_dir / "route.csv"
    write_route_csv(route_csv, samples)

    semantic_relative = os.path.relpath(semantic_path, route_dir)
    manifest = {
        "schema_version": 1,
        "route_id": route_id,
        "revision": int(revision),
        "frame_id": "map",
        "map_binding": {
            "map_id": str(map_manifest.get("map_id", "")),
            "map_version_id": str(map_manifest.get("map_version_id", "")),
            "manifest_sha256": sha256_file(map_manifest_path),
        },
        "semantic_binding": {
            "path": semantic_relative,
            "sha256": sha256_file(semantic_path),
        },
        "vehicle_binding": {
            "platform_id": str(platform["name"]),
            "platform_profile_sha256": actual_platform_hash,
        },
        "policy_binding": {
            "path": "policy.yaml",
            "sha256": sha256_file(policy_copy),
        },
        "route_csv_sha256": sha256_file(route_csv),
        "planner": {
            "backend": "semantic_boustrophedon_mvp",
            "connector_backend": "straight_candidate_only",
        },
        "status": "DRAFT",
    }
    write_route_manifest(route_dir / "route.yaml", manifest)
    return route_dir


def write_route_csv(path: str | Path, samples: Iterable[RouteSample]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=_ROUTE_FIELDS)
        writer.writeheader()
        for sample in samples:
            writer.writerow({
                "seq": int(sample.seq),
                "segment_id": sample.segment_id,
                "x": f"{sample.x:.6f}",
                "y": f"{sample.y:.6f}",
                "yaw": f"{sample.yaw:.9f}",
                "direction": sample.direction,
                "v_ref": f"{sample.v_ref:.6f}",
                "curvature": f"{sample.curvature:.9f}",
                "clearance": f"{sample.clearance:.6f}",
                "semantic_ref": sample.semantic_ref,
                "event_ref": sample.event_ref,
            })
    return path


def load_route_csv(path: str | Path) -> list[RouteSample]:
    path = Path(path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != _ROUTE_FIELDS:
            raise AssetContractError("route_csv_header_invalid", "route CSV header is not canonical")
        output = []
        for expected_seq, row in enumerate(reader):
            seq = int(row["seq"])
            if seq != expected_seq:
                raise AssetContractError("route_csv_seq_invalid", "route seq must be contiguous from zero")
            direction = str(row["direction"])
            if direction not in {"F", "R"}:
                raise AssetContractError("route_direction_invalid", "route direction must be F or R")
            sample = RouteSample(
                seq=seq,
                segment_id=str(row["segment_id"]),
                x=float(row["x"]),
                y=float(row["y"]),
                yaw=float(row["yaw"]),
                direction=direction,
                v_ref=float(row["v_ref"]),
                curvature=float(row["curvature"]),
                clearance=float(row["clearance"]),
                semantic_ref=str(row["semantic_ref"]),
                event_ref=str(row["event_ref"]),
            )
            if not all(math.isfinite(value) for value in (
                sample.x, sample.y, sample.yaw, sample.v_ref, sample.curvature, sample.clearance
            )):
                raise AssetContractError("route_numeric_invalid", "route CSV contains non-finite values")
            output.append(sample)
    if len(output) < 2:
        raise AssetContractError("route_too_short", "route CSV requires at least two rows")
    return output


def write_route_manifest(path: str | Path, value: dict) -> Path:
    path = Path(path)
    path.write_text(yaml.safe_dump(value, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return path


def _work_direction(semantic_map) -> tuple[float, float]:
    feature = next(
        (
            item for item in semantic_map.features
            if item.enabled and item.feature_type == "work_direction"
        ),
        None,
    )
    if feature is None or len(feature.coordinates) < 2:
        raise AssetContractError("route_work_direction_missing", "semantic map needs work_direction")
    first = feature.coordinates[0]
    last = feature.coordinates[-1]
    dx = float(last[0]) - float(first[0])
    dy = float(last[1]) - float(first[1])
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        raise AssetContractError("route_work_direction_invalid", "work_direction length is zero")
    return dx / length, dy / length


def _resample_polyline(points: list[list[float]], step: float) -> list[list[float]]:
    output = [list(points[0])]
    for first, second in zip(points, points[1:]):
        distance = math.dist(first, second)
        if distance <= 1e-9:
            continue
        count = max(1, int(math.ceil(distance / step)))
        for index in range(1, count + 1):
            ratio = index / count
            output.append([
                first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio,
            ])
    return output


def _samples_from_points(
    points: list[list[float]],
    segment_id: str,
    direction: str,
    speed: float,
    semantic_ref: str,
    *,
    start_seq: int,
) -> list[RouteSample]:
    output = []
    for index, point in enumerate(points):
        if index + 1 < len(points):
            target = points[index + 1]
            yaw = math.atan2(target[1] - point[1], target[0] - point[0])
        else:
            source = points[max(0, index - 1)]
            yaw = math.atan2(point[1] - source[1], point[0] - source[0])
        output.append(RouteSample(
            seq=start_seq + index,
            segment_id=segment_id,
            x=float(point[0]),
            y=float(point[1]),
            yaw=yaw,
            direction=direction,
            v_ref=speed,
            semantic_ref=semantic_ref,
        ))
    return output


def _recompute_curvature(samples: list[RouteSample]) -> list[RouteSample]:
    output = []
    for index, sample in enumerate(samples):
        curvature = 0.0
        if index > 0:
            previous = samples[index - 1]
            distance = math.hypot(sample.x - previous.x, sample.y - previous.y)
            if distance > 1e-9:
                yaw_delta = _angle_difference(sample.yaw, previous.yaw)
                curvature = yaw_delta / distance
        output.append(replace(sample, seq=index, curvature=curvature))
    return output


def _angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _dot(first, second) -> float:
    return float(first[0]) * float(second[0]) + float(first[1]) * float(second[1])
