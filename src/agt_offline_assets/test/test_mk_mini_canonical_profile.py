from pathlib import Path

import pytest

from agt_offline_assets import load_canonical_vehicle_profile, vehicle_profile_to_route_binding


def test_mk_mini_profile_is_greenhouse_preview_ready_but_not_formal_ready():
    root = Path(__file__).resolve().parents[3]
    profile = load_canonical_vehicle_profile(root / "profiles/platforms/mk_mini.yaml")

    assert profile.profile_id == "mk_mini"
    assert profile.kinematics == "ackermann"
    assert profile.physical_length_m == pytest.approx(0.840)
    assert profile.physical_width_m == pytest.approx(0.600)
    assert profile.navigation_length_m == pytest.approx(0.840)
    assert profile.navigation_width_m == pytest.approx(0.600)
    assert profile.wheel_base_m == pytest.approx(0.600)
    assert profile.track_width_m == pytest.approx(0.517)
    assert profile.wheel_diameter_m == pytest.approx(0.240)
    assert profile.ground_clearance_m == pytest.approx(0.111)
    assert profile.minimum_turning_radius_m == pytest.approx(1.500)
    assert profile.minimum_turning_radius_verified is True
    assert profile.maximum_steering_angle_deg == pytest.approx(34.0)
    assert profile.allow_in_place_rotation is False
    assert profile.planning_preview_ready is True
    assert profile.route_feasibility_ready is False

    binding = vehicle_profile_to_route_binding(profile)
    assert binding["platform_id"] == "mk_mini"
    assert binding["preview_planning_ready"] is True
    assert binding["route_feasibility_ready"] is False
    assert binding["minimum_turning_radius_m"] == pytest.approx(1.5)
    assert binding["wheel_base_m"] == pytest.approx(0.6)
