from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TURN = ROOT / "src/agt_offline_assets/agt_offline_assets/turn_zones.py"
VEHICLE = ROOT / "src/agt_offline_assets/agt_offline_assets/vehicle_profile.py"
ARCH = ROOT / "docs/architecture/agricultural_route_production.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_turn_zone_contract_is_search_envelope_not_free_space_truth():
    text = _read(TURN)
    assert 'TURN_ZONE_SCHEMA = "agt_turn_zones/v1"' in text
    assert "SEARCH_ENVELOPE_NOT_FREE_SPACE_TRUTH" in text
    assert "derive_turn_zone_candidates" in text
    assert "write_turn_zones" in text
    assert "navigation.occupancy =" not in text


def test_vehicle_adapter_reads_canonical_profile_instead_of_defining_new_truth():
    text = _read(VEHICLE)
    assert "load_canonical_vehicle_profile" in text
    assert "sha256_file(profile_path)" in text
    assert "navigation_footprint" in text
    assert "min_turning_radius_verified" in text
    assert "route_feasibility_ready" in text
    assert "profiles/platforms" in text


def test_ackermann_route_acceptance_is_fail_closed_until_radius_verified():
    text = _read(VEHICLE)
    assert 'if kinematics == "ackermann"' in text
    assert "not radius_verified" in text
    assert "acceptance_enabled = False" in text
    assert "allow_in_place = False" in text


def test_architecture_keeps_vehicle_profile_as_single_truth_source():
    text = _read(ARCH)
    assert "profiles/platforms/<platform>.yaml" in text
    assert "Aisle Graph" in text
    assert "Turn Zones" in text
    assert "Route Asset" in text
