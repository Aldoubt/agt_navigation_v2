"""Deterministic footprint and curvature validation for coverage paths."""

from dataclasses import dataclass, field
import json
import math

from shapely.geometry import Polygon, box
from shapely.strtree import STRtree


EPSILON = 1e-9


class PathValidationError(ValueError):
    """Stable input/configuration failure raised before geometric validation."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    direction: str = "UNKNOWN"


@dataclass(frozen=True)
class SampledPose:
    pose: Pose2D
    segment_index: int


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple
    frame_id: str = "map"

    def __post_init__(self):
        if self.frame_id != "map":
            raise PathValidationError("invalid_costmap_frame", "costmap frame must be map")
        if self.width <= 0 or self.height <= 0:
            raise PathValidationError(
                "invalid_costmap_dimensions", "costmap dimensions must be positive"
            )
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise PathValidationError(
                "invalid_costmap_resolution", "costmap resolution must be positive"
            )
        if not all(
            math.isfinite(value)
            for value in (self.origin_x, self.origin_y, self.origin_yaw)
        ):
            raise PathValidationError(
                "invalid_costmap_origin", "costmap origin must contain finite values"
            )
        if len(self.data) != self.width * self.height:
            raise PathValidationError(
                "invalid_costmap_data", "costmap data length does not match dimensions"
            )
        if not all(-1 <= int(value) <= 100 for value in self.data):
            raise PathValidationError(
                "invalid_costmap_value", "OccupancyGrid costs must be in [-1, 100]"
            )


@dataclass(frozen=True)
class ValidatorConfig:
    occupied_cost_threshold: int = 65
    unknown_space_policy: str = "collision"
    outside_costmap_is_collision: bool = True
    maximum_sample_count: int = 200000

    def __post_init__(self):
        if not 0 <= self.occupied_cost_threshold <= 100:
            raise PathValidationError(
                "invalid_cost_threshold", "occupied_cost_threshold must be in [0, 100]"
            )
        if self.unknown_space_policy not in {"collision", "free"}:
            raise PathValidationError(
                "invalid_unknown_space_policy",
                "unknown_space_policy must be collision or free",
            )
        if self.maximum_sample_count < 2:
            raise PathValidationError(
                "invalid_maximum_sample_count", "maximum_sample_count must be at least 2"
            )


@dataclass
class ValidationReport:
    valid: bool = False
    collision_pose_count: int = 0
    invalid_segment_indices: list = field(default_factory=list)
    maximum_cost: int = -1
    minimum_clearance: float = 0.0
    maximum_curvature: float = 0.0
    worst_curvature_segment_index: int | None = None
    worst_curvature_x: float | None = None
    worst_curvature_y: float | None = None
    worst_curvature_direction: str = "UNKNOWN"
    worst_curvature_delta_yaw: float = 0.0
    worst_curvature_chord: float = 0.0
    worst_curvature_ratio_to_limit: float | None = None
    worst_curvature_is_direction_transition: bool = False
    direction_transition_count: int = 0
    valid_cusp_count: int = 0
    ambiguous_direction_transition_count: int = 0
    direction_transition_feasible: bool = True
    direction_transition_status: str = "NONE"
    required_min_turning_radius: float = 0.0
    sample_count: int = 0
    in_place_rotation_count: int = 0
    out_of_bounds_pose_count: int = 0
    unknown_collision_pose_count: int = 0
    linear_sample_step: float = 0.0
    angular_sample_step: float = 0.0
    unknown_space_policy: str = "collision"
    error_codes: list = field(default_factory=list)
    swath_ids: list = field(default_factory=list)
    invalid_component_ids: list = field(default_factory=list)
    invalid_swath_ids: list = field(default_factory=list)
    path_fingerprint: str = ""

    def to_dict(self):
        return {
            "valid": self.valid,
            "collision_pose_count": self.collision_pose_count,
            "invalid_segment_indices": list(self.invalid_segment_indices),
            "maximum_cost": self.maximum_cost,
            "minimum_clearance": self.minimum_clearance,
            "maximum_curvature": self.maximum_curvature,
            "worst_curvature_segment_index": self.worst_curvature_segment_index,
            "worst_curvature_x_m": self.worst_curvature_x,
            "worst_curvature_y_m": self.worst_curvature_y,
            "worst_curvature_direction": self.worst_curvature_direction,
            "worst_curvature_delta_yaw_rad": self.worst_curvature_delta_yaw,
            "worst_curvature_chord_m": self.worst_curvature_chord,
            "worst_curvature_ratio_to_limit": self.worst_curvature_ratio_to_limit,
            "worst_curvature_is_direction_transition": self.worst_curvature_is_direction_transition,
            "direction_transition_count": self.direction_transition_count,
            "valid_cusp_count": self.valid_cusp_count,
            "ambiguous_direction_transition_count": self.ambiguous_direction_transition_count,
            "direction_transition_feasible": self.direction_transition_feasible,
            "direction_transition_status": self.direction_transition_status,
            "required_min_turning_radius": self.required_min_turning_radius,
            "sample_count": self.sample_count,
            "in_place_rotation_count": self.in_place_rotation_count,
            "out_of_bounds_pose_count": self.out_of_bounds_pose_count,
            "unknown_collision_pose_count": self.unknown_collision_pose_count,
            "linear_sample_step": self.linear_sample_step,
            "angular_sample_step": self.angular_sample_step,
            "unknown_space_policy": self.unknown_space_policy,
            "error_codes": list(self.error_codes),
            "swath_ids": list(self.swath_ids),
            "invalid_component_ids": list(self.invalid_component_ids),
            "invalid_swath_ids": list(self.invalid_swath_ids),
            "path_fingerprint": self.path_fingerprint,
        }

    def to_json(self):
        return json.dumps(
            self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False
        )


@dataclass
class ValidationResult:
    report: ValidationReport
    samples: list
    collision_samples: list
    invalid_samples: list


def validate_path(
    poses,
    path_frame,
    grid,
    footprint,
    min_turning_radius,
    config=None,
):
    """Validate an entire interpolated path using the full footprint polygon."""
    config = config or ValidatorConfig()
    canonical_footprint = _validate_footprint(footprint)
    path_poses = _validate_poses(poses, path_frame)
    required_radius = float(min_turning_radius)
    if not math.isfinite(required_radius) or required_radius < 0.0:
        raise PathValidationError(
            "invalid_min_turning_radius",
            "minimum turning radius must be finite and non-negative",
        )

    footprint_radius = max(math.hypot(x, y) for x, y in canonical_footprint)
    linear_step = min(grid.resolution * 0.5, footprint_radius * 0.25)
    linear_step = max(linear_step, grid.resolution * 0.05)
    angular_step = min(math.pi / 18.0, linear_step / footprint_radius)
    samples = interpolate_path(
        path_poses,
        linear_step,
        angular_step,
        maximum_sample_count=config.maximum_sample_count,
    )

    collision_index = _CollisionIndex(grid, config)
    collision_samples = []
    invalid_samples = []
    invalid_segments = set()
    error_codes = set()
    maximum_cost = -1
    minimum_clearance = math.inf
    out_of_bounds_count = 0
    unknown_collision_count = 0

    for sample in samples:
        check = collision_index.check(sample.pose, canonical_footprint)
        maximum_cost = max(maximum_cost, check.maximum_cost)
        minimum_clearance = min(minimum_clearance, check.clearance)
        if check.out_of_bounds:
            out_of_bounds_count += 1
            error_codes.add("footprint_outside_costmap")
        if check.unknown_collision:
            unknown_collision_count += 1
            error_codes.add("unknown_space_collision")
        if check.collision:
            collision_samples.append(sample)
            invalid_samples.append(sample)
            invalid_segments.add(sample.segment_index)
            error_codes.add("footprint_collision")

    maximum_curvature = 0.0
    worst_curvature = None
    in_place_rotations = 0
    direction_transition_count = 0
    valid_cusp_count = 0
    ambiguous_direction_transition_count = 0
    curvature_limit = math.inf if required_radius <= EPSILON else 1.0 / required_radius
    # Curvature uses supplied pose pairs. Interpolated collision samples are
    # straight-chord geometry and would bias finite-angle circular arcs.
    for segment_index, (previous, current) in enumerate(zip(path_poses, path_poses[1:])):
        distance = math.hypot(
            current.x - previous.x,
            current.y - previous.y,
        )
        angle = abs(_angle_difference(current.yaw, previous.yaw))
        direction_transition = {previous.direction, current.direction} == {"F", "R"}
        if direction_transition:
            direction_transition_count += 1
            if distance <= 1e-9 and angle <= 1e-6:
                valid_cusp_count += 1
            else:
                ambiguous_direction_transition_count += 1
                invalid_segments.add(segment_index)
                invalid_samples.append(SampledPose(current, segment_index))
                error_codes.add("ambiguous_direction_transition")
            # Direction changes are independent velocity-sign events. Their
            # chord is never a continuous forward/reverse motion primitive.
            continue
        curvature = _chord_curvature(angle, distance)
        maximum_curvature = max(maximum_curvature, curvature)
        direction = _direction_label(previous.direction, current.direction)
        if worst_curvature is None or curvature > worst_curvature["curvature"]:
            worst_curvature = {
                "segment_index": segment_index,
                "x": current.x,
                "y": current.y,
                "direction": direction,
                "delta_yaw": _angle_difference(current.yaw, previous.yaw),
                "chord": distance,
                "curvature": curvature,
                "transition": previous.direction != current.direction,
            }
        if distance <= EPSILON:
            if angle > EPSILON:
                in_place_rotations += 1
                if required_radius > EPSILON:
                    invalid_segments.add(segment_index)
                    invalid_samples.append(SampledPose(current, segment_index))
                    error_codes.add("minimum_turning_radius_violation")
            continue
        if curvature > curvature_limit + 1e-6:
            invalid_segments.add(segment_index)
            invalid_samples.append(SampledPose(current, segment_index))
            error_codes.add("minimum_turning_radius_violation")

    if not math.isfinite(minimum_clearance):
        minimum_clearance = 0.0
    if worst_curvature is None:
        worst_curvature = {
            "segment_index": None, "x": None, "y": None, "direction": "UNKNOWN",
            "delta_yaw": 0.0, "chord": 0.0, "curvature": 0.0, "transition": False,
        }
    ratio = (
        float(worst_curvature["curvature"]) / curvature_limit
        if math.isfinite(curvature_limit) and curvature_limit > 0.0
        else None
    )
    report = ValidationReport(
        valid=not invalid_segments,
        collision_pose_count=len(collision_samples),
        invalid_segment_indices=sorted(invalid_segments),
        maximum_cost=int(maximum_cost),
        minimum_clearance=_stable_float(minimum_clearance),
        maximum_curvature=_stable_float(maximum_curvature),
        worst_curvature_segment_index=worst_curvature["segment_index"],
        worst_curvature_x=(_stable_float(worst_curvature["x"]) if worst_curvature["x"] is not None else None),
        worst_curvature_y=(_stable_float(worst_curvature["y"]) if worst_curvature["y"] is not None else None),
        worst_curvature_direction=str(worst_curvature["direction"]),
        worst_curvature_delta_yaw=_stable_float(worst_curvature["delta_yaw"]),
        worst_curvature_chord=_stable_float(worst_curvature["chord"]),
        worst_curvature_ratio_to_limit=(_stable_float(ratio) if ratio is not None else None),
        worst_curvature_is_direction_transition=bool(worst_curvature["transition"]),
        direction_transition_count=direction_transition_count,
        valid_cusp_count=valid_cusp_count,
        ambiguous_direction_transition_count=ambiguous_direction_transition_count,
        direction_transition_feasible=ambiguous_direction_transition_count == 0,
        direction_transition_status=(
            "AMBIGUOUS_DIRECTION_TRANSITION"
            if ambiguous_direction_transition_count
            else ("VALID_CUSP" if valid_cusp_count else "NONE")
        ),
        required_min_turning_radius=_stable_float(required_radius),
        sample_count=len(samples),
        in_place_rotation_count=in_place_rotations,
        out_of_bounds_pose_count=out_of_bounds_count,
        unknown_collision_pose_count=unknown_collision_count,
        linear_sample_step=_stable_float(linear_step),
        angular_sample_step=_stable_float(angular_step),
        unknown_space_policy=config.unknown_space_policy,
        error_codes=sorted(error_codes),
    )
    return ValidationResult(
        report=report,
        samples=samples,
        collision_samples=collision_samples,
        invalid_samples=_deduplicate_samples(invalid_samples),
    )


def interpolate_path(poses, linear_step, angular_step, maximum_sample_count=200000):
    if linear_step <= 0.0 or angular_step <= 0.0:
        raise PathValidationError(
            "invalid_interpolation_step", "interpolation steps must be positive"
        )
    output = [SampledPose(poses[0], 0)]
    for segment_index, (start, end) in enumerate(zip(poses, poses[1:])):
        distance = math.hypot(end.x - start.x, end.y - start.y)
        yaw_delta = _angle_difference(end.yaw, start.yaw)
        count = max(
            1,
            int(math.ceil(distance / linear_step)),
            int(math.ceil(abs(yaw_delta) / angular_step)),
        )
        if len(output) + count > maximum_sample_count:
            raise PathValidationError(
                "maximum_sample_count_exceeded",
                "interpolated path exceeds maximum_sample_count",
            )
        for index in range(1, count + 1):
            ratio = index / count
            output.append(
                SampledPose(
                    Pose2D(
                        x=start.x + (end.x - start.x) * ratio,
                        y=start.y + (end.y - start.y) * ratio,
                        yaw=_normalize_angle(start.yaw + yaw_delta * ratio),
                        direction=(end.direction if index == count else start.direction),
                    ),
                    segment_index,
                )
            )
    return output


def footprint_shape_matches(canonical, published, tolerance=0.03):
    """Compare translation/rotation-invariant footprint shape dimensions."""
    expected = _validate_footprint(canonical)
    observed = _validate_footprint(published)
    if len(expected) != len(observed):
        return False
    expected_edges = sorted(_pairwise_distances(expected))
    observed_edges = sorted(_pairwise_distances(observed))
    return all(
        abs(first - second) <= tolerance
        for first, second in zip(expected_edges, observed_edges)
    )


@dataclass(frozen=True)
class _CellCheck:
    collision: bool
    unknown_collision: bool
    out_of_bounds: bool
    maximum_cost: int
    clearance: float


class _CollisionIndex:
    def __init__(self, grid, config):
        self.grid = grid
        self.config = config
        self.map_polygon = box(
            0.0,
            0.0,
            grid.width * grid.resolution,
            grid.height * grid.resolution,
        )
        self.obstacles = _collision_runs(grid, config)
        self.obstacle_tree = STRtree(self.obstacles) if self.obstacles else None

    def check(self, pose, footprint):
        local_pose = _world_to_map_pose(pose, self.grid)
        polygon = _transform_footprint(footprint, local_pose)
        out_of_bounds = not self.map_polygon.covers(polygon)
        collision = out_of_bounds and self.config.outside_costmap_is_collision
        unknown_collision = False
        maximum_cost = -1

        min_x, min_y, max_x, max_y = polygon.bounds
        start_x = max(0, int(math.floor(min_x / self.grid.resolution)))
        end_x = min(
            self.grid.width - 1, int(math.floor(max_x / self.grid.resolution))
        )
        start_y = max(0, int(math.floor(min_y / self.grid.resolution)))
        end_y = min(
            self.grid.height - 1, int(math.floor(max_y / self.grid.resolution))
        )
        if start_x <= end_x and start_y <= end_y:
            for row in range(start_y, end_y + 1):
                for column in range(start_x, end_x + 1):
                    cell = _cell_polygon(column, row, self.grid.resolution)
                    if not polygon.intersects(cell):
                        continue
                    cost = int(self.grid.data[row * self.grid.width + column])
                    maximum_cost = max(maximum_cost, cost)
                    if cost < 0:
                        if self.config.unknown_space_policy == "collision":
                            collision = True
                            unknown_collision = True
                    elif cost >= self.config.occupied_cost_threshold:
                        collision = True

        clearance = 0.0 if collision else polygon.distance(self.map_polygon.boundary)
        if not collision and self.obstacle_tree is not None:
            nearest = self.obstacle_tree.nearest(polygon)
            clearance = min(clearance, polygon.distance(nearest))
        return _CellCheck(
            collision=collision,
            unknown_collision=unknown_collision,
            out_of_bounds=out_of_bounds,
            maximum_cost=maximum_cost,
            clearance=clearance,
        )


def _collision_runs(grid, config):
    output = []
    for row in range(grid.height):
        start = None
        for column in range(grid.width + 1):
            colliding = False
            if column < grid.width:
                value = int(grid.data[row * grid.width + column])
                colliding = value >= config.occupied_cost_threshold or (
                    value < 0 and config.unknown_space_policy == "collision"
                )
            if colliding and start is None:
                start = column
            elif not colliding and start is not None:
                output.append(
                    box(
                        start * grid.resolution,
                        row * grid.resolution,
                        column * grid.resolution,
                        (row + 1) * grid.resolution,
                    )
                )
                start = None
    return output


def _validate_poses(poses, frame_id):
    if frame_id != "map":
        raise PathValidationError("invalid_path_frame", "path frame must be map")
    output = list(poses)
    if len(output) < 2:
        raise PathValidationError("path_too_short", "path requires at least two poses")
    for pose in output:
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            raise PathValidationError("invalid_path_pose", "path poses must be finite")
    return output


def _validate_footprint(points):
    output = [tuple(map(float, point)) for point in points]
    if len(output) > 1 and output[0] == output[-1]:
        output.pop()
    if len(output) < 3:
        raise PathValidationError(
            "invalid_platform_footprint", "footprint requires at least three points"
        )
    if not all(math.isfinite(value) for point in output for value in point):
        raise PathValidationError(
            "invalid_platform_footprint", "footprint points must be finite"
        )
    polygon = Polygon(output)
    if not polygon.is_valid or polygon.is_empty or polygon.area <= EPSILON:
        raise PathValidationError(
            "invalid_platform_footprint", "footprint polygon must be valid"
        )
    return tuple(output)


def _pairwise_distances(points):
    return [
        math.hypot(second[0] - first[0], second[1] - first[1])
        for index, first in enumerate(points)
        for second in points[index + 1:]
    ]


def _world_to_map_pose(pose, grid):
    delta_x = pose.x - grid.origin_x
    delta_y = pose.y - grid.origin_y
    cosine = math.cos(grid.origin_yaw)
    sine = math.sin(grid.origin_yaw)
    return Pose2D(
        x=cosine * delta_x + sine * delta_y,
        y=-sine * delta_x + cosine * delta_y,
        yaw=_normalize_angle(pose.yaw - grid.origin_yaw),
    )


def _transform_footprint(footprint, pose):
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Polygon(
        [
            (
                pose.x + cosine * x - sine * y,
                pose.y + sine * x + cosine * y,
            )
            for x, y in footprint
        ]
    )


def _cell_polygon(column, row, resolution):
    return box(
        column * resolution,
        row * resolution,
        (column + 1) * resolution,
        (row + 1) * resolution,
    )


def _normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _angle_difference(first, second):
    return _normalize_angle(first - second)


def _chord_curvature(delta_yaw, chord_m):
    if chord_m <= EPSILON:
        return math.inf if abs(delta_yaw) > EPSILON else 0.0
    return 2.0 * math.sin(abs(delta_yaw) / 2.0) / chord_m


def _direction_label(first, second):
    if first == second:
        return first
    if {first, second} == {"F", "R"}:
        return f"{first}/{second}"
    return second if second != "UNKNOWN" else first


def _deduplicate_samples(samples):
    output = []
    seen = set()
    for sample in samples:
        key = (
            sample.segment_index,
            round(sample.pose.x, 12),
            round(sample.pose.y, 12),
            round(sample.pose.yaw, 12),
        )
        if key not in seen:
            seen.add(key)
            output.append(sample)
    return output


def _stable_float(value):
    return float(f"{float(value):.12g}")
