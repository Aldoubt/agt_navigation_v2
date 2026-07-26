"""ROS-independent swept-footprint corridor audit."""

import math

from shapely.geometry import Polygon, box

from agt_coverage_planning.path_validator import ValidatorConfig, validate_path


def audit_corridor(poses, grid, footprint, min_turning_radius, config=None, demo_id=""):
    result = validate_path(
        poses,
        "map",
        grid,
        footprint,
        min_turning_radius,
        config or ValidatorConfig(),
    )
    conflict = int(result.report.collision_pose_count)
    occupied_cells, unknown_cells = _audited_cells(
        result.samples, grid, footprint, config or ValidatorConfig()
    )
    report = {
        "schema_version": 1,
        "demo_id": str(demo_id),
        "checked_pose_count": int(result.report.sample_count),
        "conflict_pose_count": conflict,
        "conflict_ratio": (
            conflict / result.report.sample_count if result.report.sample_count else 0.0
        ),
        "occupied_cell_count": len(occupied_cells),
        "unknown_cell_count": len(unknown_cells),
        "eligible_for_automatic_map_edit": False,
    }
    return result, report


def _audited_cells(samples, grid, footprint, config):
    occupied = set()
    unknown = set()
    origin_cosine = math.cos(grid.origin_yaw)
    origin_sine = math.sin(grid.origin_yaw)
    for sample in samples:
        delta_x = sample.pose.x - grid.origin_x
        delta_y = sample.pose.y - grid.origin_y
        map_x = origin_cosine * delta_x + origin_sine * delta_y
        map_y = -origin_sine * delta_x + origin_cosine * delta_y
        map_yaw = sample.pose.yaw - grid.origin_yaw
        cosine = math.cos(map_yaw)
        sine = math.sin(map_yaw)
        polygon = Polygon(
            [
                (
                    map_x + cosine * local_x - sine * local_y,
                    map_y + sine * local_x + cosine * local_y,
                )
                for local_x, local_y in footprint
            ]
        )
        min_x, min_y, max_x, max_y = polygon.bounds
        start_x = max(0, int(math.floor(min_x / grid.resolution)))
        end_x = min(grid.width - 1, int(math.floor(max_x / grid.resolution)))
        start_y = max(0, int(math.floor(min_y / grid.resolution)))
        end_y = min(grid.height - 1, int(math.floor(max_y / grid.resolution)))
        for row in range(start_y, end_y + 1):
            for column in range(start_x, end_x + 1):
                cell = box(
                    column * grid.resolution,
                    row * grid.resolution,
                    (column + 1) * grid.resolution,
                    (row + 1) * grid.resolution,
                )
                if not polygon.intersects(cell):
                    continue
                value = int(grid.data[row * grid.width + column])
                if value < 0:
                    unknown.add((column, row))
                elif value >= config.occupied_cost_threshold:
                    occupied.add((column, row))
    return occupied, unknown
