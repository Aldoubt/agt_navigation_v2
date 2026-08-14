from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
ORDERING = ROOT / "src/agt_offline_assets/agt_offline_assets/agricultural_coverage_ordering.py"
MK_MINI = ROOT / "profiles/platforms/mk_mini.yaml"
ARCH = ROOT / "docs/architecture/agricultural_route_production.md"
PLATFORM_DOC = ROOT / "docs/interfaces/mk_mini_chassis_route_profile.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_greenhouse_route_production_binds_mk_mini_not_bunker():
    profile = yaml.safe_load(_read(MK_MINI))["platform"]
    assert profile["name"] == "mk_mini"
    assert profile["kinematics"] == "ackermann"
    assert profile["deployment"]["primary_scene"] == "greenhouse"
    assert profile["geometry"]["min_turning_radius_verified"] is True
    assert profile["geometry"]["min_turning_radius"] == 1.5
    doc = _read(PLATFORM_DOC)
    assert "Greenhouse execution / aisle-route production" in doc
    assert "GAAS open-field experiments" in doc
    assert "BUNKER tracked chassis" in doc


def test_mk_mini_manufacturer_geometry_is_frozen_for_preview():
    profile = yaml.safe_load(_read(MK_MINI))["platform"]
    geometry = profile["geometry"]
    assert geometry["length"] == 0.840
    assert geometry["width"] == 0.600
    assert geometry["height"] == 0.310
    assert geometry["wheel_base"] == 0.600
    assert geometry["track_width"] == 0.517
    assert geometry["wheel_diameter"] == 0.240
    assert geometry["ground_clearance"] == 0.111
    assert geometry["max_steering_angle_deg"] == 34.0
    assert profile["route_acceptance"]["preview_planning_enabled"] is True
    assert profile["route_acceptance"]["enabled"] is False


def test_coverage_ordering_keeps_orientation_separate_from_reverse_motion():
    code = _read(ORDERING)
    for token in (
        "DETERMINISTIC_BOUSTROPHEDON",
        "VEHICLE_WIDTH_INFEASIBLE",
        "motion_direction=\"FORWARD\"",
        "WITH_ROW_DIRECTION",
        "AGAINST_ROW_DIRECTION",
        "turn_low_u",
        "turn_high_u",
        "ConnectorRequest",
    ):
        assert token in code
    assert 'motion_direction="REVERSE"' not in code


def test_architecture_names_scene_specific_platform_ownership():
    arch = _read(ARCH)
    assert "MK-mini" in arch
    assert "BUNKER" in arch
    assert "GNSS" in arch or "RTK" in arch
