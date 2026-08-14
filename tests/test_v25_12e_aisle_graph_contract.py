from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
AISLE_GRAPH = ROOT / "src/agt_offline_assets/agt_offline_assets/agricultural_aisle_graph.py"
OFFLINE_INIT = ROOT / "src/agt_offline_assets/agt_offline_assets/__init__.py"
REVIEW = ROOT / "src/agt_map_workbench/agt_map_workbench/review_workbench.py"
ARCH = ROOT / "docs/architecture/agricultural_route_production.md"
STAGE = ROOT / "docs/v2.5/V25_12E_AGRICULTURAL_ROUTE_PRODUCTION.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_aisle_graph_is_an_offline_intermediate_asset():
    graph = _read(AISLE_GRAPH)
    for token in (
        'AISLE_GRAPH_SCHEMA = "agt_agricultural_aisle_graph/v1"',
        "class AisleGraphConfig",
        "class AislePrimitive",
        "class AgriculturalAisleGraph",
        "derive_agricultural_aisle_graph",
        "write_agricultural_aisle_graph",
        'status: str = "DRAFT"',
    ):
        assert token in graph
    assert "result.occupancy =" not in graph
    assert "RouteSample" not in graph


def test_public_api_exports_aisle_graph_contract():
    init = _read(OFFLINE_INIT)
    for token in (
        "AISLE_GRAPH_SCHEMA",
        "AisleGraphConfig",
        "AislePrimitive",
        "AgriculturalAisleGraph",
        "derive_agricultural_aisle_graph",
        "write_agricultural_aisle_graph",
    ):
        assert token in init


def test_workbench_exports_draft_graph_through_offline_api():
    review = _read(REVIEW)
    assert 'addMenu("离线资产")' in review
    assert 'addAction("导出 Aisle Graph YAML")' in review
    assert "derive_agricultural_aisle_graph(" in review
    assert "write_agricultural_aisle_graph(" in review
    assert '"DRAFT aisles:' in review
    assert '"pcd_name"' in review
    assert "/home/yangxuan" not in review


def test_architecture_and_stage_ledger_exist():
    architecture = _read(ARCH)
    stage = _read(STAGE)
    assert "Aisle Graph" in architecture
    assert "Coverage Ordering" in architecture
    assert "Dubins / Dubins-CC forward first" in stage
    assert "Reeds-Shepp reverse fallback" in stage
    assert "Smac Hybrid-A* / State Lattice" in stage
    assert "R1 — Aisle Graph deterministic export" in stage
