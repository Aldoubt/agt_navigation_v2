"""Advisory-only safety audit for offline coverage preview paths."""

from dataclasses import dataclass

from .path_validator import (
    GridMap,
    PathValidationError,
    ValidatorConfig,
    validate_path,
)


@dataclass(frozen=True)
class PreviewAuditResult:
    report: dict
    collision_samples: tuple


def merge_preview_grids(base_grid, keepout_grid, occupied_cost_threshold=65):
    """Merge a base OccupancyGrid and semantic mask without losing unknown cells."""
    _require_matching_grids(base_grid, keepout_grid)
    threshold = int(occupied_cost_threshold)
    data = []
    for base_value, keepout_value in zip(base_grid.data, keepout_grid.data):
        if base_value >= threshold or keepout_value >= threshold:
            data.append(max(base_value, keepout_value))
        elif base_value < 0 or keepout_value < 0:
            data.append(-1)
        else:
            data.append(max(base_value, keepout_value))
    return GridMap(
        width=base_grid.width,
        height=base_grid.height,
        resolution=base_grid.resolution,
        origin_x=base_grid.origin_x,
        origin_y=base_grid.origin_y,
        origin_yaw=base_grid.origin_yaw,
        data=tuple(data),
        frame_id=base_grid.frame_id,
    )


def audit_preview_path(
    poses,
    base_grid,
    keepout_grid,
    footprint,
    min_turning_radius,
    config=None,
):
    """Audit a candidate path while keeping the result permanently non-actionable."""
    config = config or ValidatorConfig()
    combined_grid = merge_preview_grids(
        base_grid, keepout_grid, config.occupied_cost_threshold
    )
    combined = validate_path(
        poses,
        "map",
        combined_grid,
        footprint,
        min_turning_radius,
        config,
    )
    collision_only_config = ValidatorConfig(
        occupied_cost_threshold=config.occupied_cost_threshold,
        unknown_space_policy=config.unknown_space_policy,
        outside_costmap_is_collision=config.outside_costmap_is_collision,
        maximum_sample_count=config.maximum_sample_count,
    )
    base = validate_path(
        poses, "map", base_grid, footprint, 0.0, collision_only_config
    )
    keepout = validate_path(
        poses, "map", keepout_grid, footprint, 0.0, collision_only_config
    )
    report = {
        "schema_version": "1.0",
        "status": "CONFLICT" if combined.report.error_codes else "CLEAR",
        "advisory_only": True,
        "eligible_for_execution": False,
        "source_topic": "/agt/coverage/path_preview",
        "sample_count": combined.report.sample_count,
        "collision_pose_count": combined.report.collision_pose_count,
        "base_collision_pose_count": base.report.collision_pose_count,
        "keepout_collision_pose_count": keepout.report.collision_pose_count,
        "invalid_segment_count": len(combined.report.invalid_segment_indices),
        "turning_radius_violation": (
            "minimum_turning_radius_violation" in combined.report.error_codes
        ),
        "required_min_turning_radius": combined.report.required_min_turning_radius,
        "minimum_clearance": combined.report.minimum_clearance,
        "error_codes": list(combined.report.error_codes),
    }
    return PreviewAuditResult(report, tuple(combined.collision_samples))


def _require_matching_grids(first, second):
    fields = (
        "width",
        "height",
        "resolution",
        "origin_x",
        "origin_y",
        "origin_yaw",
        "frame_id",
    )
    if any(getattr(first, field) != getattr(second, field) for field in fields):
        raise PathValidationError(
            "preview_grid_metadata_mismatch",
            "base map and semantic keepout mask metadata must match exactly",
        )
