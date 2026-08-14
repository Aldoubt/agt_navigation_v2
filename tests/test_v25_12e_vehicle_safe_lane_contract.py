from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py"
ARCH = ROOT / "docs/architecture/agricultural_route_vehicle_safe_lane.md"
STATE = ROOT / "docs/v2.5/V25_12E_CURRENT_STATE.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_vehicle_safe_lane_schema_and_statuses_exist():
    text = _read(MODULE)
    assert 'VEHICLE_SAFE_LANE_SCHEMA = "agt_vehicle_safe_aisle_lane/v1"' in text
    assert "VEHICLE_SAFE_LANE_READY" in text
    assert "VEHICLE_SAFE_LANE_PARTIAL" in text
    assert "NO_VEHICLE_SAFE_LANE" in text
    assert "PREVIEW_ONLY_NOT_R8_VEHICLE_READY" in text


def test_vehicle_safe_lane_is_navigation_and_graph_immutable():
    text = _read(MODULE)
    architecture = _read(ARCH)
    assert "Aisle Graph          not modified" in architecture
    assert "Navigation occupancy not modified" in architecture
    assert "navigation.occupancy =" not in text
    assert "apply_navigation_overrides" not in text
    assert "force_free" not in text


def test_vehicle_safe_lane_has_bounded_lateral_continuity():
    text = _read(MODULE)
    architecture = _read(ARCH)
    assert "maximum_lateral_shift_m" in text
    assert "maximum_lateral_step_m" in text
    assert "breaks the lane segment" in architecture
    assert "never teleport laterally" in architecture


def test_current_state_records_real_anchor_smoke_and_lane_gate():
    state = _read(STATE)
    assert "2  READY_FOR_LOCAL_CONNECTOR" in state
    assert "11 HOLD_ANCHOR_REVIEW" in state
    assert "agt_vehicle_safe_aisle_lane/v1" in state
    assert "Vehicle-Safe Aisle Lane" in state
