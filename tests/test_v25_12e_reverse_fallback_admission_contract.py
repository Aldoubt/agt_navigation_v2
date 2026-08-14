from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ADMISSION = ROOT / "src/agt_offline_assets/agt_offline_assets/reverse_fallback_admission.py"
R6_DOC = ROOT / "docs/v2.5/V25_12E_R6_REVERSE_FALLBACK.md"
EXPERIMENT = ROOT / "docs/experiments/v25_12e_r5_6_forward_candidate_audit_20260814.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_r6a_auto_admission_is_evidence_gated():
    text = _read(ADMISSION)
    for token in (
        "agt_reverse_fallback_admission/v1",
        "AUTO_ONLY_LOCAL_FORWARD_OCCUPANCY_BLOCKED",
        "ELIGIBLE_REVERSE_FALLBACK",
        "HOLD_MAP_REVIEW",
        "HOLD_MIXED_EVIDENCE",
        "map_insufficient_never_auto_admitted",
        "R6B_INPUT_SELECTION_NOT_ROUTE_READY",
    ):
        assert token in text


def test_r6_docs_preserve_forward_reverse_search_boundary():
    doc = _read(R6_DOC)
    experiment = _read(EXPERIMENT)
    assert "13 LOCAL_FORWARD_OCCUPANCY_BLOCKED" in doc
    assert "1  LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT" in doc
    assert "3  LOCAL_FORWARD_MIXED_EVIDENCE" in doc
    assert "Reeds-Shepp / reverse-aware local connector" in doc
    assert "Smac Hybrid-A* / State Lattice search fallback" in doc
    assert "R6B shall consume only connector IDs admitted by R6A" in doc
    assert "eligible reverse fallback 13" in doc
    assert "connector_015" in experiment
    assert "connector_017" in experiment
    assert "R6 reverse planning must not consume all 17 connectors" in experiment
