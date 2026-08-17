from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Sequence

from shapely.geometry import Polygon

from .contracts import PathPoint


def _transform_footprint(footprint, x: float, y: float, yaw: float) -> Polygon:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return Polygon(
        [
            (x + c * local_x - s * local_y, y + s * local_x + c * local_y)
            for local_x, local_y in footprint
        ]
    )


def _interpolate_yaw(a: float, b: float, ratio: float) -> float:
    delta = math.atan2(math.sin(b - a), math.cos(b - a))
    return a + ratio * delta


def _sample_path(points: Sequence[PathPoint], step_m: float):
    if not math.isfinite(step_m) or step_m <= 0.0:
        raise ValueError("semantic sample_step_m must be positive and finite")
    if not points:
        return
    yield points[0].x_m, points[0].y_m, points[0].yaw_rad
    for first, second in zip(points, points[1:]):
        distance = math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
        count = max(1, int(math.ceil(distance / step_m)))
        for index in range(1, count + 1):
            ratio = index / count
            yield (
                first.x_m + ratio * (second.x_m - first.x_m),
                first.y_m + ratio * (second.y_m - first.y_m),
                _interpolate_yaw(first.yaw_rad, second.yaw_rad, ratio),
            )


def _load_hard_semantic_regions(path: Path):
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("type") != "FeatureCollection":
        raise ValueError("semantic map must be a GeoJSON FeatureCollection")
    if str(document.get("frame_id", "")) != "map":
        raise ValueError("semantic map frame_id must be map")

    fields = []
    forbidden = []
    for feature in document.get("features", []):
        if not isinstance(feature, dict):
            continue
        properties = feature.get("properties") or {}
        if properties.get("enabled", True) is False:
            continue
        feature_type = str(properties.get("feature_type", ""))
        if feature_type not in {"field_boundary", "exclusion_zone", "keepout_zone"}:
            continue
        geometry = feature.get("geometry") or {}
        if geometry.get("type") != "Polygon":
            raise ValueError(f"{feature_type} must use Polygon geometry")
        coordinates = geometry.get("coordinates") or []
        if not coordinates or len(coordinates[0]) < 4:
            raise ValueError(f"{feature_type} polygon is invalid")
        polygon = Polygon(coordinates[0], coordinates[1:])
        if not polygon.is_valid or polygon.is_empty:
            raise ValueError(f"{feature_type} polygon is invalid")
        feature_id = str(properties.get("id", feature.get("id", "")))
        if feature_type == "field_boundary":
            fields.append((feature_id, polygon))
        else:
            code = "EXCLUSION" if feature_type == "exclusion_zone" else "KEEP_OUT"
            forbidden.append((code, feature_id, polygon))
    if not fields:
        raise ValueError("semantic path validation requires at least one enabled field_boundary")
    return fields, forbidden


def evaluate_semantic_path(
    points: Sequence[PathPoint],
    semantic_map_path: Path | str,
    platform_profile,
    *,
    sample_step_m: float = 0.10,
) -> dict:
    """Evaluate hard spatial semantic constraints using the full robot footprint.

    A violation episode starts when the footprint first leaves every enabled field
    boundary or intersects an enabled exclusion/keepout zone, and ends only when all
    hard semantic constraints are satisfied again. This keeps the episode metric
    independent of the route's native sampling density; the violating sample count is
    emitted separately for diagnostics.
    """
    path = Path(semantic_map_path).expanduser().resolve()
    fields, forbidden = _load_hard_semantic_regions(path)
    footprint = tuple((float(x), float(y)) for x, y in platform_profile.navigation_footprint)
    if len(footprint) < 3:
        raise ValueError("platform navigation footprint requires at least three vertices")

    episodes = 0
    violating_samples = 0
    in_violation = False
    codes_seen: set[str] = set()
    feature_ids_seen: set[str] = set()
    sample_count = 0

    for x, y, yaw in _sample_path(points, sample_step_m):
        sample_count += 1
        robot = _transform_footprint(footprint, x, y, yaw)
        sample_codes: set[str] = set()
        sample_feature_ids: set[str] = set()

        if not any(field.covers(robot) for _, field in fields):
            sample_codes.add("OUTSIDE_FIELD")
        for code, feature_id, region in forbidden:
            if robot.intersects(region):
                sample_codes.add(code)
                if feature_id:
                    sample_feature_ids.add(feature_id)

        violating = bool(sample_codes)
        if violating:
            violating_samples += 1
            codes_seen.update(sample_codes)
            feature_ids_seen.update(sample_feature_ids)
            if not in_violation:
                episodes += 1
        in_violation = violating

    return {
        "semantic_violation_count": episodes,
        "semantic_violating_sample_count": violating_samples,
        "semantic_validation_sample_count": sample_count,
        "semantic_violation_codes": sorted(codes_seen),
        "semantic_violated_feature_ids": sorted(feature_ids_seen),
        "hard_semantic_feasible": episodes == 0,
    }
