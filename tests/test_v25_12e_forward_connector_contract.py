from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FORWARD = ROOT / "src/agt_offline_assets/agt_offline_assets/forward_connector.py"
INIT = ROOT / "src/agt_offline_assets/agt_offline_assets/__init__.py"
IO = ROOT / "src/agt_offline_assets/agt_offline_assets/agricultural_route_io.py"
ARCH = ROOT / "docs/architecture/agricultural_route_production.md"
STAGE = ROOT / "docs/v2.5/V25_12E_AGRICULTURAL_ROUTE_PRODUCTION.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_forward_connector_is_offline_r5_centerline_gate():
    source = _read(FORWARD)
    for token in (
        "agt_forward_connector_plan/v1",
        "ANALYTIC_DUBINS_FORWARD_ONLY",
        "ACCEPTED_CENTERLINE",
        "NO_FORWARD_DUBINS_IN_TURN_ZONE",
        "CENTERLINE_KINEMATICS_AND_TURN_ZONE_ONLY",
        "minimum_turning_radius_m",
    ):
        assert token in source
    for family in ("LSL", "RSR", "LSR", "RSL", "RLR", "LRL"):
        assert family in source
    assert "result.occupancy =" not in source
    assert "route_feasibility_ready = True" not in source


def test_forward_connector_api_and_frozen_io_are_exported():
    init = _read(INIT)
    io = _read(IO)
    for token in (
        "ForwardConnectorConfig",
        "ForwardConnectorPlan",
        "derive_forward_connector_plan",
        "forward_connector_plan_to_dict",
        "load_coverage_connector_requests",
        "load_turn_zones",
        "write_forward_connector_plan",
    ):
        assert token in init
    assert "write_forward_connector_plan" in io
    assert "load_coverage_connector_requests" in io
    assert "load_turn_zones" in io


def test_architecture_keeps_forward_then_reverse_then_search_fallback():
    architecture = _read(ARCH)
    stage = _read(STAGE)
    assert "Forward-first" in architecture
    assert "Reverse fallback" in architecture
    assert "Smac Hybrid" in architecture
    assert "R5" in stage
    assert "R6" in stage
    assert "R7" in stage
