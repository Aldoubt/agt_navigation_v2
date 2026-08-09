"""Canonical Reeds-Shepp shortest-path solver.

The family equations are adapted from PythonRobotics' Reeds-Shepp planner:
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py
Copyright (c) Atsushi Sakai and contributors, MIT License.  The implementation
below is self-contained and uses no PythonRobotics runtime dependency.
"""

from dataclasses import dataclass
import math

from .connector import ConnectorPlannerBackend, ConnectorRequest, ConnectorResult, ConnectorSample


_EPS = 1.0e-9


def normalize_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class ReedsSheppPrimitive:
    kind: str
    length: float


@dataclass(frozen=True)
class ReedsSheppPath:
    primitives: tuple[ReedsSheppPrimitive, ...]
    total_length_m: float

    @property
    def reverse_length_m(self) -> float:
        return sum(abs(item.length) for item in self.primitives if item.length < 0.0)

    @property
    def direction_changes(self) -> int:
        directions = [item.length >= 0.0 for item in self.primitives]
        return sum(first != second for first, second in zip(directions, directions[1:]))

    @property
    def primitive_signature(self) -> str:
        return " ".join(f"{item.kind}{'+' if item.length >= 0.0 else '-'}" for item in self.primitives)


def _mod2pi(value: float) -> float:
    return normalize_angle(value)


def _polar(x: float, y: float) -> tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


# The following equations are the complete canonical family set. Symmetry
# transforms in _generate_paths provide the remaining reflected/time-reversed
# configurations without duplicating the equations.
def _lsl(x, y, phi):
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    v = _mod2pi(phi - t)
    return (0.0 <= t <= math.pi and 0.0 <= v <= math.pi), [t, u, v], "LSL"


def _lsr(x, y, phi):
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u1 * u1 < 4.0:
        return False, [], "LSR"
    u = math.sqrt(u1 * u1 - 4.0)
    theta = math.atan2(2.0, u)
    t = _mod2pi(t1 + theta)
    v = _mod2pi(t - phi)
    return t >= 0.0 and v >= 0.0, [t, u, v], "LSR"


def _lrl(x, y, phi):
    zeta, eta = x - math.sin(phi), y - 1.0 + math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 > 4.0:
        return False, [], "LRL"
    a = math.acos(max(-1.0, min(1.0, u1 / 4.0)))
    t = _mod2pi(a + theta + math.pi / 2.0)
    u = _mod2pi(math.pi - 2.0 * a)
    v = _mod2pi(phi - t - u)
    return True, [t, -u, v], "LRL"


def _lrl_time(x, y, phi):
    zeta, eta = x - math.sin(phi), y - 1.0 + math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 > 4.0:
        return False, [], "LRL"
    a = math.acos(max(-1.0, min(1.0, u1 / 4.0)))
    t = _mod2pi(a + theta + math.pi / 2.0)
    u = _mod2pi(math.pi - 2.0 * a)
    v = _mod2pi(-phi + t + u)
    return True, [t, -u, -v], "LRL"


def _lr_l(x, y, phi):
    zeta, eta = x - math.sin(phi), y - 1.0 + math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 > 4.0 or u1 < _EPS:
        return False, [], "LRL"
    u = math.acos(max(-1.0, min(1.0, 1.0 - u1 * u1 / 8.0)))
    a = math.asin(max(-1.0, min(1.0, 2.0 * math.sin(u) / u1)))
    t = _mod2pi(-a + theta + math.pi / 2.0)
    v = _mod2pi(t - u - phi)
    return t >= 0.0 and v >= 0.0, [t, u, -v], "LRL"


def _lrlr(x, y, phi):
    zeta, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 > 2.0:
        return False, [], "LRLR"
    a = math.acos(max(-1.0, min(1.0, (u1 + 2.0) / 4.0)))
    t = _mod2pi(theta + a + math.pi / 2.0)
    u = _mod2pi(a)
    v = _mod2pi(phi - t + 2.0 * u)
    return t >= 0.0 and u >= 0.0 and v >= 0.0, [t, u, -u, -v], "LRLR"


def _lrlr_time(x, y, phi):
    zeta, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    u1, theta = _polar(zeta, eta)
    u2 = (20.0 - u1 * u1) / 16.0
    if not 0.0 <= u2 <= 1.0 or u1 < _EPS:
        return False, [], "LRLR"
    u = math.acos(u2)
    a = math.asin(max(-1.0, min(1.0, 2.0 * math.sin(u) / u1)))
    t = _mod2pi(theta + a + math.pi / 2.0)
    v = _mod2pi(t - phi)
    return t >= 0.0 and v >= 0.0, [t, -u, -u, v], "LRLR"


def _lrs_l(x, y, phi):
    zeta, eta = x - math.sin(phi), y - 1.0 + math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 < 2.0:
        return False, [], "LRSL"
    root = math.sqrt(u1 * u1 - 4.0)
    u = root - 2.0
    a = math.atan2(2.0, root)
    t = _mod2pi(theta + a + math.pi / 2.0)
    v = _mod2pi(t - phi + math.pi / 2.0)
    return t >= 0.0 and v >= 0.0, [t, -math.pi / 2.0, -u, -v], "LRSL"


def _lsrl(x, y, phi):
    zeta, eta = x - math.sin(phi), y - 1.0 + math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 < 2.0:
        return False, [], "LSRL"
    root = math.sqrt(u1 * u1 - 4.0)
    u = root - 2.0
    a = math.atan2(root, 2.0)
    t = _mod2pi(theta - a + math.pi / 2.0)
    v = _mod2pi(t - phi - math.pi / 2.0)
    return t >= 0.0 and v >= 0.0, [t, u, math.pi / 2.0, -v], "LSRL"


def _lrsr(x, y, phi):
    zeta, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 < 2.0:
        return False, [], "LRSR"
    t = _mod2pi(theta + math.pi / 2.0)
    u = u1 - 2.0
    v = _mod2pi(phi - t - math.pi / 2.0)
    return t >= 0.0 and v >= 0.0, [t, -math.pi / 2.0, -u, -v], "LRSR"


def _lslr(x, y, phi):
    zeta, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 < 2.0:
        return False, [], "LSLR"
    t = _mod2pi(theta)
    u = u1 - 2.0
    v = _mod2pi(phi - t - math.pi / 2.0)
    return t >= 0.0 and v >= 0.0, [t, u, math.pi / 2.0, -v], "LSLR"


def _lrs_lr(x, y, phi):
    zeta, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    u1, theta = _polar(zeta, eta)
    if u1 < 4.0:
        return False, [], "LRSLR"
    root = math.sqrt(u1 * u1 - 4.0)
    u = root - 4.0
    a = math.atan2(2.0, root)
    t = _mod2pi(theta + a + math.pi / 2.0)
    v = _mod2pi(t - phi)
    return t >= 0.0 and v >= 0.0, [t, -math.pi / 2.0, -u, -math.pi / 2.0, v], "LRSLR"


_FAMILIES = (_lsl, _lsr, _lrl, _lrl_time, _lr_l, _lrlr, _lrlr_time,
             _lrs_l, _lsrl, _lrsr, _lslr, _lrs_lr)


def _reflect(kinds: str) -> str:
    return "".join("R" if kind == "L" else "L" if kind == "R" else "S" for kind in kinds)


def _timeflip(lengths):
    return [-value for value in lengths]


def _generate_paths(x, y, phi, rho, allow_reverse):
    paths = []
    for family in _FAMILIES:
        transforms = (
            (x, y, phi, False, False),
            (-x, y, -phi, True, False),
            (x, -y, -phi, False, True),
            (-x, -y, phi, True, True),
        )
        for tx, ty, tphi, flip_time, reflect in transforms:
            valid, lengths, signature = family(tx, ty, tphi)
            if not valid:
                continue
            if flip_time:
                lengths = _timeflip(lengths)
            if reflect:
                signature = _reflect(signature)
            if not allow_reverse and any(length < -_EPS for length in lengths):
                continue
            primitives = tuple(
                ReedsSheppPrimitive(kind, length * rho)
                for kind, length in zip(signature, lengths)
                if abs(length * rho) >= _EPS
            )
            if primitives:
                paths.append(ReedsSheppPath(primitives, sum(abs(item.length) for item in primitives)))
    return paths


def _path_key(path: ReedsSheppPath):
    return (round(path.total_length_m, 9), path.direction_changes,
            len(path.primitives), path.primitive_signature)


def sample_reeds_shepp(path: ReedsSheppPath, start_pose, turning_radius_m, resolution_m):
    x, y, yaw = map(float, start_pose)
    output = []
    previous_direction = None
    for primitive in path.primitives:
        direction = "F" if primitive.length > 0.0 else "R"
        if previous_direction != direction:
            output.append(ConnectorSample(x, y, normalize_angle(yaw), direction))
            previous_direction = direction
        count = max(1, int(math.ceil(abs(primitive.length) / resolution_m)))
        step = primitive.length / count
        for _ in range(count):
            if primitive.kind == "S":
                x += step * math.cos(yaw)
                y += step * math.sin(yaw)
            else:
                radius = turning_radius_m
                curvature = (1.0 if primitive.kind == "L" else -1.0) / radius
                delta = curvature * step
                old_yaw = yaw
                yaw += delta
                x += (math.sin(yaw) - math.sin(old_yaw)) / curvature
                y += (-math.cos(yaw) + math.cos(old_yaw)) / curvature
            output.append(ConnectorSample(x, y, normalize_angle(yaw), direction))
    return output


def solve_reeds_shepp(request: ConnectorRequest) -> tuple[ReedsSheppPath | None, str]:
    rho = float(request.min_turning_radius_m)
    if not math.isfinite(rho) or rho <= 0.0:
        return None, "REEDS_SHEPP_INVALID_TURNING_RADIUS"
    values = (*request.start_pose, *request.goal_pose, request.path_resolution_m)
    if not all(math.isfinite(float(value)) for value in values):
        return None, "REEDS_SHEPP_NON_FINITE_POSE"
    if (
        math.hypot(request.goal_pose[0] - request.start_pose[0], request.goal_pose[1] - request.start_pose[1]) <= _EPS
        and abs(normalize_angle(request.goal_pose[2] - request.start_pose[2])) <= _EPS
    ):
        return ReedsSheppPath((), 0.0), ""
    dx = request.goal_pose[0] - request.start_pose[0]
    dy = request.goal_pose[1] - request.start_pose[1]
    c, s = math.cos(request.start_pose[2]), math.sin(request.start_pose[2])
    x = (c * dx + s * dy) / rho
    y = (-s * dx + c * dy) / rho
    phi = normalize_angle(request.goal_pose[2] - request.start_pose[2])
    candidates = _generate_paths(x, y, phi, rho, request.allow_reverse)
    if not candidates:
        return None, "REEDS_SHEPP_FORWARD_ONLY_NO_SOLUTION" if not request.allow_reverse else "REEDS_SHEPP_NO_SOLUTION"
    return min(candidates, key=_path_key), ""


class ReedsSheppConnectorBackend(ConnectorPlannerBackend):
    name = "reeds_shepp"

    def plan(self, request: ConnectorRequest, context=None) -> ConnectorResult:
        if request.path_resolution_m <= 0.0 or not math.isfinite(request.path_resolution_m):
            return ConnectorResult((), self.name, False, "REEDS_SHEPP_INVALID_PATH_RESOLUTION")
        path, failure = solve_reeds_shepp(request)
        if path is None:
            return ConnectorResult((), self.name, False, failure)
        if not path.primitives:
            return ConnectorResult((ConnectorSample(*request.start_pose, "F"),), self.name, True)
        try:
            samples = sample_reeds_shepp(
                path, request.start_pose, request.min_turning_radius_m, request.path_resolution_m
            )
            if not samples:
                return ConnectorResult((), self.name, False, "REEDS_SHEPP_SAMPLING_FAILED")
            samples[0] = ConnectorSample(*request.start_pose, samples[0].direction)
            samples[-1] = ConnectorSample(*request.goal_pose, samples[-1].direction)
            return ConnectorResult(tuple(samples), self.name, True)
        except (ArithmeticError, ValueError, RuntimeError) as exc:
            return ConnectorResult((), self.name, False, f"REEDS_SHEPP_SAMPLING_FAILED: {exc}")
