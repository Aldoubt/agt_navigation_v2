from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GATE = ROOT / "src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py"
STAGE = ROOT / "docs/v2.5/V25_12E_AGRICULTURAL_ROUTE_PRODUCTION.md"
ARCH = ROOT / "docs/architecture/agricultural_route_production.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_connector_navigation_gate_is_path_specific_preview_evidence():
    gate = _read(GATE)
    for token in (
        "agt_forward_connector_navigation_gate/v1",
        "derive_forward_connector_navigation_gate",
        "NavigationGridEvidence",
        "navigation_footprint_xy",
        "PREVIEW_FOOTPRINT_FREE",
        "NO_FORWARD_PREVIEW_FREE_CANDIDATE",
        "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
        "_dubins_candidates",
        "_sample_candidate",
    ):
        assert token in gate
    assert "navigation.occupancy =" not in gate
    assert "apply_navigation_overrides" not in gate
    assert "route_acceptance_enabled =" not in gate


def test_docs_reject_whole_shared_turn_zone_free_ratio_as_path_gate():
    stage = _read(STAGE)
    arch = _read(ARCH)
    assert "FREE                       968   / 0.164" in stage
    assert "OCCUPIED                  4904   / 0.832" in stage
    assert "FREE                      1251   / 0.344" in stage
    assert "OCCUPIED                  2357   / 0.649" in stage
    assert "whole-zone added FREE ratio is too conservative" in stage
    assert "Connector-specific Navigation Preview Gate" in arch
    assert "shared-zone FREE ratio is review evidence only" in arch
    assert "Reeds-Shepp reverse fallback" in stage
