from pathlib import Path

import yaml

from agt_offline_assets import (
    load_canonical_vehicle_profile,
    vehicle_profile_to_route_binding,
)


def _write(path: Path, payload) -> Path:
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path


def test_tracked_profile_uses_navigation_footprint_and_zero_radius(tmp_path: Path):
    path = _write(
        tmp_path / "tracked.yaml",
        {
            "platform": {
                "name": "tracked_test",
                "kinematics": "tracked_differential",
                "footprint_frame": "base_footprint",
                "base_frame": "base_link",
                "geometry": {
                    "length": 1.0,
                    "width": 0.8,
                    "footprint": [[0.5, 0.4], [0.5, -0.4], [-0.5, -0.4], [-0.5, 0.4]],
                    "navigation_footprint": [[0.6, 0.5], [0.6, -0.5], [-0.6, -0.5], [-0.6, 0.5]],
                },
                "limits": {
                    "max_forward_velocity": 0.5,
                    "max_reverse_velocity": 0.25,
                    "max_angular_velocity": 0.6,
                },
                "coverage_repair": {"allow_in_place_rotation": True},
            }
        },
    )
    profile = load_canonical_vehicle_profile(path)
    assert profile.profile_id == "tracked_test"
    assert profile.navigation_width_m == 1.0
    assert profile.minimum_turning_radius_m == 0.0
    assert profile.minimum_turning_radius_verified is True
    assert profile.allow_in_place_rotation is True
    assert profile.route_feasibility_ready is True
    binding = vehicle_profile_to_route_binding(profile)
    assert binding["platform_id"] == "tracked_test"
    assert binding["platform_profile_sha256"].startswith("sha256:")


def test_ackermann_profile_fails_closed_without_verified_turning_radius(tmp_path: Path):
    path = _write(
        tmp_path / "ackermann.yaml",
        {
            "platform": {
                "name": "ackermann_test",
                "kinematics": "ackermann",
                "geometry": {
                    "length": 0.7,
                    "width": 0.55,
                    "footprint": [[0.35, 0.275], [0.35, -0.275], [-0.35, -0.275], [-0.35, 0.275]],
                    "min_turning_radius_verified": False,
                    "min_turning_radius": 0.0,
                },
                "route_acceptance": {
                    "enabled": False,
                    "blocked_reason": "turn radius not measured",
                },
            }
        },
    )
    profile = load_canonical_vehicle_profile(path)
    assert profile.kinematics == "ackermann"
    assert profile.minimum_turning_radius_verified is False
    assert profile.allow_in_place_rotation is False
    assert profile.route_feasibility_ready is False
    assert "turn radius" in profile.blocked_reason


def test_ackermann_profile_can_become_route_ready_when_radius_is_verified(tmp_path: Path):
    path = _write(
        tmp_path / "ackermann_ready.yaml",
        {
            "platform": {
                "name": "ackermann_ready",
                "kinematics": "ackermann",
                "geometry": {
                    "length": 0.8,
                    "width": 0.6,
                    "footprint": [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
                    "min_turning_radius_verified": True,
                    "min_turning_radius": 1.2,
                },
                "route_acceptance": {"enabled": True},
            }
        },
    )
    profile = load_canonical_vehicle_profile(path)
    assert profile.minimum_turning_radius_m == 1.2
    assert profile.minimum_turning_radius_verified is True
    assert profile.route_feasibility_ready is True
