from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / "src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py"
OVERLAY = ROOT / "src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py"
PANEL = ROOT / "src/agt_map_workbench/agt_map_workbench/route_debug_panel.py"
SPEC = ROOT / "docs/superpowers/specs/2026-08-15-route-debug-2d-design.md"
STATE = ROOT / "docs/v2.5/V25_12E_CURRENT_STATE.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_route_debug_remains_render_only_and_not_a_planner():
    overlay = _read(OVERLAY)
    panel = _read(PANEL)
    assert "agt_route_debug_overlay/v1" in overlay
    assert "DEBUG_RENDER_ONLY" in overlay
    assert "route_debug_overlay.geojson" in _read(SPEC)
    assert "derive_reverse_primitive_connector_plan" not in panel
    assert "derive_forward_connector_plan" not in panel
    assert "derive_ground_relative_navigation_map" not in panel
    assert "force_free" not in panel.lower()


def test_route_debug_preserves_no_go_and_reverse_semantics():
    dataset = _read(DATASET)
    state = _read(STATE)
    assert "no_go" in dataset
    assert "BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP" in state
    assert "Route Debug 2D" in state
    assert "CORE IMPLEMENTED / LOCAL ACCEPTANCE PENDING" in state
    assert "No production map/route semantics changed" in state
