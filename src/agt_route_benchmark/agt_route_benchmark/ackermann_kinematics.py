from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AckermannState:
    x_m: float
    y_m: float
    yaw_rad: float


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def step_ackermann_twist(
    state: AckermannState,
    *,
    linear_velocity_mps: float,
    angular_velocity_rps: float,
    dt_s: float,
    wheel_base_m: float,
    min_turning_radius_m: float,
) -> tuple[AckermannState, dict]:
    """Integrate a rear-axle Ackermann/bicycle state from a Twist-equivalent command.

    Nav2 RPP publishes linear velocity and yaw rate. For an Ackermann chassis their
    ratio defines path curvature. This model clamps that curvature to the accepted
    minimum turning radius and explicitly rejects rotate-in-place motion. The arc
    integration is exact for constant velocity/curvature over the time step.
    """
    values = (
        state.x_m,
        state.y_m,
        state.yaw_rad,
        linear_velocity_mps,
        angular_velocity_rps,
        dt_s,
        wheel_base_m,
        min_turning_radius_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Ackermann state and inputs must be finite")
    if dt_s < 0.0:
        raise ValueError("dt_s must be non-negative")
    if wheel_base_m <= 0.0 or min_turning_radius_m <= 0.0:
        raise ValueError("wheel_base_m and min_turning_radius_m must be positive")

    velocity = float(linear_velocity_mps)
    if abs(velocity) <= 1e-9 or dt_s == 0.0:
        return state, {
            "commanded_curvature_1pm": 0.0,
            "effective_curvature_1pm": 0.0,
            "effective_yaw_rate_rps": 0.0,
            "steering_angle_rad": 0.0,
            "curvature_clamped": abs(float(angular_velocity_rps)) > 1e-9,
        }

    commanded_curvature = float(angular_velocity_rps) / velocity
    curvature_limit = 1.0 / float(min_turning_radius_m)
    effective_curvature = max(-curvature_limit, min(curvature_limit, commanded_curvature))
    yaw_rate = velocity * effective_curvature
    steering_angle = math.atan(float(wheel_base_m) * effective_curvature)
    yaw0 = float(state.yaw_rad)
    dt = float(dt_s)

    if abs(yaw_rate) <= 1e-12:
        x = state.x_m + velocity * math.cos(yaw0) * dt
        y = state.y_m + velocity * math.sin(yaw0) * dt
        yaw = yaw0
    else:
        yaw_unwrapped = yaw0 + yaw_rate * dt
        radius_signed = velocity / yaw_rate
        x = state.x_m + radius_signed * (math.sin(yaw_unwrapped) - math.sin(yaw0))
        y = state.y_m - radius_signed * (math.cos(yaw_unwrapped) - math.cos(yaw0))
        yaw = _wrap_angle(yaw_unwrapped)

    return AckermannState(float(x), float(y), float(yaw)), {
        "commanded_curvature_1pm": commanded_curvature,
        "effective_curvature_1pm": effective_curvature,
        "effective_yaw_rate_rps": yaw_rate,
        "steering_angle_rad": steering_angle,
        "curvature_clamped": abs(effective_curvature - commanded_curvature) > 1e-12,
    }
