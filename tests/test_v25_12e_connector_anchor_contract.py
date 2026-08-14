from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "src/agt_offline_assets/agt_offline_assets/connector_anchors.py"
R6_DOC = ROOT / "docs/v2.5/V25_12E_R6_REVERSE_FALLBACK.md"
ARCH = ROOT / "docs/architecture/agricultural_route_connector_anchors.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_safe_connector_anchor_contract_exists():
    module = _read(MODULE)
    assert 'CONNECTOR_ANCHOR_SCHEMA = "agt_connector_anchor_plan/v1"' in module
    assert "NEAREST_STABLE_FREE_POSE_INWARD_ON_EXISTING_AISLE_CENTERLINE" in module
    assert "TRIM_AISLE_TRAVERSAL_TO_SELECTED_CONNECTOR_ANCHOR" in module
    assert "READY_FOR_LOCAL_CONNECTOR" in module
    assert "HOLD_ANCHOR_REVIEW" in module


def test_anchor_layer_preserves_immutable_map_and_graph_boundary():
    module = _read(MODULE)
    architecture = _read(ARCH)
    assert "Aisle Graph          not modified" in architecture
    assert "Navigation occupancy not modified" in architecture
    assert "navigation.occupancy =" not in module
    assert "READY Route" not in module


def test_r6_document_keeps_long_term_reverse_backend_wording_and_real_smoke():
    doc = _read(R6_DOC)
    assert "Reeds-Shepp / reverse-aware local connector" in doc
    assert "11 R6B_START_FOOTPRINT_NOT_FREE" in doc
    assert "connector_015" in doc
    assert "connector_017" in doc
    assert "raw structural centerline endpoint" in doc
