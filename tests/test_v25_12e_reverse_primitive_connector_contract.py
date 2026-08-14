from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R6B = ROOT / "src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py"
R6 = ROOT / "docs/v2.5/V25_12E_R6_REVERSE_FALLBACK.md"
ARCH = ROOT / "docs/architecture/agricultural_route_production.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_r6b_is_explicitly_bounded_and_not_mislabeled_reeds_shepp():
    text = _read(R6B)
    for token in (
        "agt_reverse_primitive_connector_plan/v1",
        "BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP",
        "R6A_ELIGIBLE_CONNECTOR_IDS_ONLY",
        "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
        "max_cusps",
        "motion_direction",
        "is_cusp",
        "minimum_turning_radius_m",
    ):
        assert token in text


def test_r6b_fails_closed_on_navigation_evidence_and_does_not_mutate_map():
    text = _read(R6B)
    assert "np.all(cells == FREE)" in text
    assert "navigation.occupancy =" not in text
    assert "apply_navigation_overrides" not in text
    assert "Route READY" not in text


def test_r6_docs_keep_local_reverse_separate_from_r7_search():
    stage = _read(R6)
    architecture = _read(ARCH)
    assert "R6A real-data acceptance PASS" in stage
    assert "13" in stage
    assert "BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP" in stage
    assert "R7 Smac Hybrid-A* / State Lattice" in stage
    assert "Reeds-Shepp" in architecture
