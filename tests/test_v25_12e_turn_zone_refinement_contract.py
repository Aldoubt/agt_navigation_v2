from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GRID = ROOT / "src/agt_offline_assets/agt_offline_assets/navigation_grid.py"
REFINE = ROOT / "src/agt_offline_assets/agt_offline_assets/turn_zone_refinement.py"
STAGE = ROOT / "docs/v2.5/V25_12E_AGRICULTURAL_ROUTE_PRODUCTION.md"
ARCH = ROOT / "docs/architecture/agricultural_route_production.md"
EXPERIMENT = ROOT / "docs/experiments/v25_12e_r5_turn_zone_fit_20260814.md"


def _read(path):
    return path.read_text(encoding="utf-8")


def test_frozen_navigation_grid_is_lightweight_route_evidence():
    text = _read(GRID)
    assert "NavigationGridEvidence" in text
    assert "load_navigation_grid" in text
    assert "np.flipud" in text
    assert "NavigationMapResult" in text
    assert "does not reconstruct" in text


def test_turn_zone_refinement_is_proposal_not_drive_permission():
    text = _read(REFINE)
    for token in (
        "agt_turn_zone_refinement_proposal/v1",
        "derive_turn_zone_refinement_proposal",
        "EVIDENCE_SUPPORTS_EXPANSION",
        "REVIEW_REQUIRED_OUT_OF_GRID",
        "REVIEW_REQUIRED_NAVIGATION_CONFLICT",
        "PROPOSAL_NOT_DRIVE_PERMISSION",
    ):
        assert token in text
    assert "proposed & ~current" in text
    assert "navigation.occupancy =" not in text
    assert "apply_navigation_overrides" not in text


def test_real_zone_fit_record_and_architecture_preserve_r6_decision_gate():
    stage = _read(STAGE)
    arch = _read(ARCH)
    experiment = _read(EXPERIMENT)
    assert "connector_016" in stage
    assert "0.846 m" in stage
    assert "connector_001" in stage
    assert "0.277 m" in stage
    assert "Turn-Zone Refinement Proposal" in arch
    assert "NavigationGridEvidence" in arch
    assert "Do not start R6 for all 17 connectors yet" in experiment
    assert "Dubins / Dubins-CC forward first" in stage
    assert "Reeds-Shepp reverse fallback" in stage
