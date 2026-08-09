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
    gx, gy = world_to_grid(context, sample.x, sample.y)
    if gx < 0 or gy < 0 or gx >= grid.width or gy >= grid.height:
        return False
    cell = grid.data[gy * grid.width + gx]
    if cell < 0 and not context.unknown_space_allowed:
        return False
    if cell >= 100:
        return False
    c, s = math.cos(sample.yaw), math.sin(sample.yaw)
    footprint_points = [(sample.x + c * px - s * py, sample.y + s * px + c * py) for px, py in context.footprint]
    for x, y in footprint_points + [(sample.x, sample.y)]:
        px, py = world_to_grid(context, x, y)
        if px < 0 or py < 0 or px >= grid.width or py >= grid.height:
            return False
        value = grid.data[py * grid.width + px]
        if value >= 100 or (value < 0 and not context.unknown_space_allowed):
            return False
        if context.field_boundaries and not any(_point_in_polygon(x, y, polygon) for polygon in context.field_boundaries):
            return False
        if any(_point_in_polygon(x, y, polygon) for polygon in context.semantic_keepouts):
            return False
    return True


def _motion(start, kind, direction, length, radius, resolution, context):
    count = max(1, int(math.ceil(abs(length) / max(resolution, context.map_resolution_m / 2.0))))
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
        start_key = (*start_grid, yaw_to_bin(request.start_pose[2], bins), "F")
        start = HybridNode(*request.start_pose, *start_grid, start_key[2], "F", 0.0, 0.0, None, ())
        nodes = {start_key: start}
        queue = [(0.0, 0.0, 0.0, start_grid[0], start_grid[1], start_key[2], "F", start_key)]
        directions = ("F", "R") if request.allow_reverse else ("F",)
        began = time.monotonic()
        while queue:
            if len(nodes) >= max_iterations:
                return ConnectorResult((), self.name, False, "HYBRID_ASTAR_MAX_ITERATIONS")
            if time.monotonic() - began > timeout:
                return ConnectorResult((), self.name, False, "HYBRID_ASTAR_TIMEOUT")
            _, _, _, _, _, _, _, key = heapq.heappop(queue)
            current = nodes[key]
            rs_path, _ = solve_reeds_shepp(
                ConnectorRequest((current.x, current.y, current.yaw), request.goal_pose, request.path_resolution_m, radius, request.allow_reverse)
            )
            if rs_path is not None:
                rs_samples = sample_reeds_shepp(rs_path, (current.x, current.y, current.yaw), radius, request.path_resolution_m)
                if rs_samples and all(check_pose_traversable(context, sample) for sample in rs_samples):
                    prefix = _reconstruct(nodes, key)
                    return ConnectorResult(tuple(prefix + list(rs_samples)), self.name, True)
            for kind in ("L", "S", "R"):
                for direction in directions:
                    signed_length = primitive_length if direction == "F" else -primitive_length
                    motion = _motion((current.x, current.y, current.yaw), kind, direction, signed_length, radius, request.path_resolution_m, context)
                    if not motion:
                        continue
                    end = motion[-1]
                    grid_key = (*world_to_grid(context, end.x, end.y), yaw_to_bin(end.yaw, bins), direction)
                    if grid_key in nodes and nodes[grid_key].g <= current.g + abs(signed_length):
                        continue
                    g = current.g + abs(signed_length) * (float(options.get("reverse_penalty", 2.0)) if direction == "R" else 1.0)
                    h = _heuristic(context, end, request.goal_pose, radius, obstacle_distances)
                    child = HybridNode(end.x, end.y, end.yaw, grid_key[0], grid_key[1], grid_key[2], direction, g, h, key, motion)
                    nodes[grid_key] = child
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
