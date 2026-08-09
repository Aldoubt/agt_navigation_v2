"""Bounded local Hybrid A* connector for offline agricultural headlands."""

from dataclasses import dataclass
import heapq
import math
import time

from .connector import (
    ConnectorPlannerBackend,
    ConnectorPlanningContext,
    ConnectorRequest,
    ConnectorResult,
    ConnectorSample,
)
from .reeds_shepp import normalize_angle, solve_reeds_shepp, sample_reeds_shepp
from .obstacle_heuristic import build_obstacle_heuristic
from agt_coverage_planning.path_validator import Pose2D, ValidatorConfig, check_footprint_pose


@dataclass(frozen=True)
class HybridNode:
    x: float
    y: float
    yaw: float
    grid_x: int
    grid_y: int
    yaw_bin: int
    direction: str
    g: float
    h: float
    parent_key: tuple | None
    motion: tuple[ConnectorSample, ...]


def world_to_grid(context: ConnectorPlanningContext, x: float, y: float) -> tuple[int, int]:
    ox, oy, oyaw = context.map_origin
    dx, dy = x - ox, y - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    return math.floor(local_x / context.map_resolution_m), math.floor(local_y / context.map_resolution_m)


def grid_to_world(context: ConnectorPlanningContext, grid_x: int, grid_y: int) -> tuple[float, float]:
    ox, oy, oyaw = context.map_origin
    local_x = (grid_x + 0.5) * context.map_resolution_m
    local_y = (grid_y + 0.5) * context.map_resolution_m
    c, s = math.cos(oyaw), math.sin(oyaw)
    return ox + c * local_x - s * local_y, oy + s * local_x + c * local_y


def yaw_to_bin(yaw: float, angle_bins: int) -> int:
    value = (normalize_angle(yaw) + math.pi) / (2.0 * math.pi)
    return int(math.floor(value * angle_bins + 0.5)) % angle_bins


def bin_to_yaw(yaw_bin: int, angle_bins: int) -> float:
    return normalize_angle((int(yaw_bin) % angle_bins) * 2.0 * math.pi / angle_bins - math.pi)


def _point_in_polygon(x, y, polygon):
    inside = False
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        if (first[1] > y) != (second[1] > y):
            cross = (second[0] - first[0]) * (y - first[1]) / (second[1] - first[1]) + first[0]
            if x < cross:
                inside = not inside
    return inside


def check_pose_traversable(context: ConnectorPlanningContext, sample: ConnectorSample) -> bool:
    grid = context.occupancy_grid
    config = ValidatorConfig(
        unknown_space_policy="free" if context.unknown_space_allowed else "collision",
        outside_costmap_is_collision=True,
    )
    if not check_footprint_pose(grid, context.footprint, Pose2D(sample.x, sample.y, sample.yaw), config):
        return False
    c, s = math.cos(sample.yaw), math.sin(sample.yaw)
    footprint_points = [(sample.x + c * px - s * py, sample.y + s * px + c * py) for px, py in context.footprint]
    min_x = min(point[0] for point in footprint_points)
    max_x = max(point[0] for point in footprint_points)
    min_y = min(point[1] for point in footprint_points)
    max_y = max(point[1] for point in footprint_points)
    semantic_points = list(footprint_points) + [(sample.x, sample.y)]
    step = max(context.footprint_check_resolution_m, 1.0e-6)
    x = min_x
    while x <= max_x + 1.0e-9:
        y = min_y
        while y <= max_y + 1.0e-9:
            if _point_in_polygon(x, y, footprint_points + [footprint_points[0]]):
                semantic_points.append((x, y))
            y += step
        x += step
    if context.field_boundaries and not all(
        any(_point_in_polygon(x, y, polygon) for polygon in context.field_boundaries)
        for x, y in semantic_points
    ):
        return False
    if any(_point_in_polygon(x, y, polygon) for x, y in semantic_points for polygon in context.semantic_keepouts):
        return False
    return True


def _motion(start, kind, direction, length, radius, resolution, context):
    collision_step = min(context.footprint_check_resolution_m, context.map_resolution_m / 2.0)
    count = max(1, int(math.ceil(abs(length) / collision_step)))
    ds = length / count
    x, y, yaw = start
    output = []
    curvature = 0.0 if kind == "S" else (1.0 if kind == "L" else -1.0) / radius
    for _ in range(count):
        if kind == "S":
            x += ds * math.cos(yaw)
            y += ds * math.sin(yaw)
        else:
            old_yaw = yaw
            delta = curvature * ds
            yaw = normalize_angle(yaw + delta)
            x += (math.sin(yaw) - math.sin(old_yaw)) / curvature
            y += (-math.cos(yaw) + math.cos(old_yaw)) / curvature
        sample = ConnectorSample(x, y, yaw, direction)
        if not check_pose_traversable(context, sample):
            return None
        output.append(sample)
    return tuple(output)


def _heuristic(context, node, goal, radius, obstacle_distances):
    rs, _ = solve_reeds_shepp(ConnectorRequest((node.x, node.y, node.yaw), goal, context.map_resolution_m, radius, True))
    rs_distance = rs.total_length_m if rs is not None else math.hypot(goal[0] - node.x, goal[1] - node.y)
    grid_distance = obstacle_distances.get(world_to_grid(context, node.x, node.y))
    obstacle_distance = math.inf if grid_distance is None else grid_distance * context.map_resolution_m
    return max(rs_distance, obstacle_distance)


class HybridAStarConnectorBackend(ConnectorPlannerBackend):
    name = "hybrid_astar"

    def plan(self, request: ConnectorRequest, context=None) -> ConnectorResult:
        if not isinstance(context, ConnectorPlanningContext):
            return ConnectorResult((), self.name, False, "HYBRID_ASTAR_CONTEXT_MISSING")
        grid = context.occupancy_grid
        if grid.width <= 0 or grid.height <= 0 or context.map_resolution_m <= 0.0:
            return ConnectorResult((), self.name, False, "HYBRID_ASTAR_INVALID_GRID")
        radius = float(request.min_turning_radius_m)
        if not math.isfinite(radius) or radius <= 0.0:
            return ConnectorResult((), self.name, False, "REEDS_SHEPP_INVALID_TURNING_RADIUS")
        if not check_pose_traversable(context, ConnectorSample(*request.start_pose, "F")):
            return ConnectorResult((), self.name, False, "HYBRID_ASTAR_START_COLLISION")
        if not check_pose_traversable(context, ConnectorSample(*request.goal_pose, "F")):
            return ConnectorResult((), self.name, False, "HYBRID_ASTAR_GOAL_COLLISION")
        if request.path_resolution_m <= 0.0 or not math.isfinite(request.path_resolution_m):
            return ConnectorResult((), self.name, False, "HYBRID_ASTAR_INVALID_GRID")

        options = dict(context.options or {})
        defaults = {
            "angle_bins": 72,
            "reverse_penalty": 2.0,
            "direction_change_penalty": 0.2,
            "non_straight_penalty": 1.05,
            "max_iterations": 100000,
            "max_planning_time_s": 3.0,
            "analytic_expansion": True,
            "analytic_expansion_ratio": 3.5,
            "analytic_expansion_max_length_factor": 5.0,
            "planning_window_margin_m": max(5.0 * radius, 2.0 * math.hypot(*context.footprint[0])),
        }
        for name, value in defaults.items():
            options.setdefault(name, value)
        if (
            int(options["angle_bins"]) <= 0
            or int(options["max_iterations"]) <= 0
            or float(options["max_planning_time_s"]) <= 0.0
            or float(options["reverse_penalty"]) < 1.0
            or float(options["non_straight_penalty"]) < 1.0
            or float(options["direction_change_penalty"]) < 0.0
            or float(options["analytic_expansion_ratio"]) <= 0.0
            or float(options["analytic_expansion_max_length_factor"]) <= 0.0
        ):
            return ConnectorResult((), self.name, False, "HYBRID_ASTAR_INVALID_CONFIG")
        bins = int(options.get("angle_bins", 72))
        if bins <= 0:
            return ConnectorResult((), self.name, False, "HYBRID_ASTAR_INVALID_GRID")
        max_iterations = int(options.get("max_iterations", 100000))
        timeout = float(options.get("max_planning_time_s", 3.0))
        angle_size = 2.0 * math.pi / bins
        primitive_length = max(context.map_resolution_m, radius * angle_size)
        start_grid = world_to_grid(context, *request.start_pose[:2])
        goal_grid = world_to_grid(context, *request.goal_pose[:2])
        obstacle_distances = build_obstacle_heuristic(context, goal_grid)
        margin = float(options["planning_window_margin_m"])
        window = (
            min(request.start_pose[0], request.goal_pose[0]) - margin,
            max(request.start_pose[0], request.goal_pose[0]) + margin,
            min(request.start_pose[1], request.goal_pose[1]) - margin,
            max(request.start_pose[1], request.goal_pose[1]) + margin,
        )
        start_key = (*start_grid, yaw_to_bin(request.start_pose[2], bins), "F")
        start = HybridNode(*request.start_pose, *start_grid, start_key[2], "F", 0.0, 0.0, None, ())
        nodes = {start_key: start}
        queue = [(0.0, 0.0, 0.0, start_grid[0], start_grid[1], start_key[2], "F", start_key)]
        directions = ("F", "R") if request.allow_reverse else ("F",)
        began = time.monotonic()
        expanded_nodes = 0
        generated_nodes = 1
        rs_backend = None
        if bool(options["analytic_expansion"]):
            from .reeds_shepp import ReedsSheppConnectorBackend

            rs_backend = ReedsSheppConnectorBackend()
        while queue:
            if expanded_nodes >= max_iterations:
                return ConnectorResult((), self.name, False, "HYBRID_ASTAR_MAX_ITERATIONS")
            if time.monotonic() - began > timeout:
                return ConnectorResult((), self.name, False, "HYBRID_ASTAR_TIMEOUT")
            _, _, g_snapshot, _, _, _, _, key = heapq.heappop(queue)
            current = nodes[key]
            if abs(g_snapshot - current.g) > 1.0e-9:
                continue
            expanded_nodes += 1
            distance_to_goal = math.hypot(request.goal_pose[0] - current.x, request.goal_pose[1] - current.y)
            if rs_backend is not None and expanded_nodes % 8 == 0 and distance_to_goal <= float(options["analytic_expansion_ratio"]) * radius:
                analytic = rs_backend.plan(
                    ConnectorRequest((current.x, current.y, current.yaw), request.goal_pose, request.path_resolution_m, radius, request.allow_reverse)
                )
                if analytic.success and len(analytic.samples) > 0:
                    analytic_length = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(analytic.samples, analytic.samples[1:]))
                    if analytic_length <= float(options["analytic_expansion_max_length_factor"]) * radius and all(check_pose_traversable(context, sample) for sample in analytic.samples):
                        prefix = _reconstruct(nodes, key)
                        return ConnectorResult(tuple(prefix + list(analytic.samples)), self.name, True)
            for kind in ("L", "S", "R"):
                for direction in directions:
                    signed_length = primitive_length if direction == "F" else -primitive_length
                    motion = _motion((current.x, current.y, current.yaw), kind, direction, signed_length, radius, request.path_resolution_m, context)
                    if not motion:
                        continue
                    if any(not (window[0] <= sample.x <= window[1] and window[2] <= sample.y <= window[3]) for sample in motion):
                        continue
                    end = motion[-1]
                    grid_key = (*world_to_grid(context, end.x, end.y), yaw_to_bin(end.yaw, bins), direction)
                    if grid_key in nodes and nodes[grid_key].g <= current.g + abs(signed_length):
                        continue
                    motion_penalty = float(options["reverse_penalty"]) if direction == "R" else 1.0
                    if kind != "S":
                        motion_penalty *= float(options["non_straight_penalty"])
                    if direction != current.direction:
                        motion_penalty += float(options["direction_change_penalty"]) / abs(signed_length)
                    g = current.g + abs(signed_length) * motion_penalty
                    h = _heuristic(context, end, request.goal_pose, radius, obstacle_distances)
                    child = HybridNode(end.x, end.y, end.yaw, grid_key[0], grid_key[1], grid_key[2], direction, g, h, key, motion)
                    nodes[grid_key] = child
                    generated_nodes += 1
                    heapq.heappush(queue, (g + h, h, g, grid_key[0], grid_key[1], grid_key[2], direction, grid_key))
        return ConnectorResult((), self.name, False, "HYBRID_ASTAR_NO_PATH")


def _reconstruct(nodes, key):
    edges = []
    while key is not None:
        node = nodes[key]
        edges.append(node.motion)
        key = node.parent_key
    root = next(node for node in nodes.values() if node.parent_key is None)
    output = [ConnectorSample(root.x, root.y, root.yaw, root.direction)]
    for edge in reversed(edges[:-1]):
        output.extend(edge)
    return output
