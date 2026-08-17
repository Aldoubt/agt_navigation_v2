from __future__ import annotations

from pathlib import Path
from typing import Sequence

from .contracts import PathPoint
from .map_io import Nav2Map, nav2_map_occupancy_data
from .semantic_path_validation import evaluate_semantic_path


def evaluate_normalized_path(
    points: Sequence[PathPoint],
    nav_map: Nav2Map,
    platform_profile,
    *,
    semantic_map_path: Path | str | None = None,
) -> dict:
    """Evaluate one normalized path using shared geometric/kinematic/semantic gates."""
    try:
        from agt_coverage_planning.path_validator import GridMap, Pose2D, ValidatorConfig, validate_path
    except ImportError as exc:
        raise RuntimeError(
            "full-footprint validation requires agt_coverage_planning.path_validator"
        ) from exc

    height, width = nav_map.image.shape[:2]
    grid = GridMap(
        width=int(width),
        height=int(height),
        resolution=float(nav_map.resolution_m),
        origin_x=float(nav_map.origin[0]),
        origin_y=float(nav_map.origin[1]),
        origin_yaw=float(nav_map.origin[2]),
        data=nav2_map_occupancy_data(nav_map),
        frame_id="map",
    )
    poses = tuple(
        Pose2D(float(point.x_m), float(point.y_m), float(point.yaw_rad))
        for point in points
    )
    result = validate_path(
        poses,
        "map",
        grid,
        platform_profile.navigation_footprint,
        float(platform_profile.min_turning_radius_m),
        config=ValidatorConfig(),
    )

    report = result.report
    error_codes = tuple(str(code) for code in report.error_codes)
    error_set = set(error_codes)
    radius = float(platform_profile.min_turning_radius_m)
    curvature_limit = 1.0 / radius
    validated_curvature = float(report.maximum_curvature)
    collision_free = (
        int(report.collision_pose_count) == 0
        and int(report.out_of_bounds_pose_count) == 0
        and int(report.unknown_collision_pose_count) == 0
        and "footprint_collision" not in error_set
        and "footprint_outside_costmap" not in error_set
        and "unknown_space_collision" not in error_set
    )
    kinematic_feasible = "minimum_turning_radius_violation" not in error_set

    metrics = {
        "execution_feasible": bool(report.valid),
        "collision_free": collision_free,
        "kinematic_feasible": kinematic_feasible,
        "footprint_collision_count": int(report.collision_pose_count),
        "min_clearance_m": float(report.minimum_clearance),
        "validated_max_abs_curvature_1pm": validated_curvature,
        "required_max_curvature_1pm": curvature_limit,
        "curvature_excess_1pm": max(0.0, validated_curvature - curvature_limit),
        "in_place_rotation_count": int(report.in_place_rotation_count),
        "validation_sample_count": int(report.sample_count),
        "validation_error_codes": list(error_codes),
    }
    if semantic_map_path is not None:
        semantic_metrics = evaluate_semantic_path(
            points,
            semantic_map_path,
            platform_profile,
            sample_step_m=min(0.10, float(nav_map.resolution_m)),
        )
        metrics.update(semantic_metrics)
        metrics["execution_feasible"] = bool(
            metrics["execution_feasible"] and semantic_metrics["hard_semantic_feasible"]
        )
    return metrics
