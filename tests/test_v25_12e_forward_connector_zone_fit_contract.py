from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DIAG = ROOT / "src/agt_offline_assets/agt_offline_assets/forward_connector_diagnostics.py"
STAGE = ROOT / "docs/v2.5/V25_12E_AGRICULTURAL_ROUTE_PRODUCTION.md"
ARCH = ROOT / "docs/architecture/agricultural_route_production.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_forward_zone_fit_diagnostic_exists_without_changing_r5_acceptance_scope():
    diagnostic = _read(DIAG)
    for token in (
        "agt_forward_connector_zone_fit_diagnostic/v1",
        "diagnose_forward_connector_zone_fit",
        "required_outward_extension_m",
        "required_inward_extension_m",
        "required_lateral_low_extension_m",
        "required_lateral_high_extension_m",
        "ROW_FRAME_ENVELOPE_ONLY_NOT_COLLISION_TRUTH",
    ):
        assert token in diagnostic


def test_docs_require_diagnosis_before_reverse_fallback():
    stage = _read(STAGE)
    architecture = _read(ARCH)
    assert "Dubins / Dubins-CC forward first" in stage
    assert "17/17" in architecture
    assert "Turn-Zone fit diagnostic" in architecture
    assert "only unresolved physically forward-infeasible connectors enter R6" in stage
    assert "Reeds-Shepp reverse fallback" in stage
    assert "Navigation Map" in architecture
    assert "不是 collision truth" in stage
