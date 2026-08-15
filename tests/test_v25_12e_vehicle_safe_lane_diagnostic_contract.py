from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane_diagnostics.py"
STATE = ROOT / "docs/v2.5/V25_12E_CURRENT_STATE.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_lane_diagnostic_is_explanatory_not_a_route_override():
    text = _read(MODULE)
    assert 'VEHICLE_SAFE_LANE_DIAGNOSTIC_SCHEMA = "agt_vehicle_safe_lane_diagnostic/v1"' in text
    assert "EXPLAIN_VEHICLE_SAFE_LANE_FAILURE_ONLY" in text
    assert "PREVIEW_ONLY_NOT_R8_VEHICLE_READY" in text
    assert "navigation.occupancy =" not in text
    assert "apply_navigation_overrides" not in text
    assert "force_free" not in text


def test_lane_diagnostic_distinguishes_center_map_and_footprint_failures():
    text = _read(MODULE)
    for token in (
        "CENTER_REFERENCE_OCCUPIED_DOMINANT",
        "CENTER_REFERENCE_UNKNOWN_DOMINANT",
        "FOOTPRINT_OCCUPIED_DOMINANT",
        "FOOTPRINT_UNKNOWN_DOMINANT",
        "MOSTLY_CONFIGURATION_SPACE_FREE",
        "STRUCTURAL_WIDTH_BLOCKED",
    ):
        assert token in text
    assert "least total non-FREE evidence" in text


def test_current_state_freezes_real_lane_failure_before_more_r6_search():
    state = _read(STATE)
    assert "1  VEHICLE_SAFE_LANE_READY" in state
    assert "7  VEHICLE_SAFE_LANE_PARTIAL" in state
    assert "11 NO_VEHICLE_SAFE_LANE" in state
    assert "Do not continue R6B or R7 yet" in state
