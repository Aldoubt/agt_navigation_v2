import math

from agt_route_benchmark.ackermann_kinematics import AckermannState, step_ackermann_twist


def test_zero_linear_velocity_cannot_rotate_ackermann_in_place():
    state = AckermannState(0.0, 0.0, 0.3)
    next_state, diag = step_ackermann_twist(
        state,
        linear_velocity_mps=0.0,
        angular_velocity_rps=1.0,
        dt_s=1.0,
        wheel_base_m=0.6,
        min_turning_radius_m=1.5,
    )
    assert next_state == state
    assert diag["effective_yaw_rate_rps"] == 0.0
    assert diag["steering_angle_rad"] == 0.0


def test_curvature_is_clamped_by_minimum_turning_radius():
    state = AckermannState(0.0, 0.0, 0.0)
    next_state, diag = step_ackermann_twist(
        state,
        linear_velocity_mps=1.0,
        angular_velocity_rps=2.0,
        dt_s=1.0,
        wheel_base_m=0.6,
        min_turning_radius_m=1.5,
    )
    assert abs(diag["effective_curvature_1pm"] - 1.0 / 1.5) < 1e-12
    assert abs(diag["effective_yaw_rate_rps"] - 1.0 / 1.5) < 1e-12
    assert abs(diag["steering_angle_rad"] - math.atan(0.6 / 1.5)) < 1e-12
    assert next_state.x_m > 0.0
    assert next_state.y_m > 0.0


def test_reverse_motion_preserves_commanded_path_curvature_sign():
    state = AckermannState(0.0, 0.0, 0.0)
    next_state, diag = step_ackermann_twist(
        state,
        linear_velocity_mps=-0.5,
        angular_velocity_rps=-0.2,
        dt_s=1.0,
        wheel_base_m=0.6,
        min_turning_radius_m=1.5,
    )
    assert diag["effective_curvature_1pm"] > 0.0
    assert diag["effective_yaw_rate_rps"] < 0.0
    assert next_state.x_m < 0.0
    assert next_state.yaw_rad < 0.0


def test_straight_motion_integrates_exactly():
    state = AckermannState(1.0, 2.0, math.pi / 2.0)
    next_state, diag = step_ackermann_twist(
        state,
        linear_velocity_mps=0.3,
        angular_velocity_rps=0.0,
        dt_s=2.0,
        wheel_base_m=0.6,
        min_turning_radius_m=1.5,
    )
    assert abs(next_state.x_m - 1.0) < 1e-12
    assert abs(next_state.y_m - 2.6) < 1e-12
    assert abs(next_state.yaw_rad - math.pi / 2.0) < 1e-12
    assert diag["effective_curvature_1pm"] == 0.0
