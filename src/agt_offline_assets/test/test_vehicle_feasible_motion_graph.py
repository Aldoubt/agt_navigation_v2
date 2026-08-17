import importlib


def test_a3_public_module_exists_with_frozen_schema():
    module = importlib.import_module("agt_offline_assets.vehicle_feasible_motion_graph")
    assert module.VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA == "agt_vehicle_feasible_motion_graph/v1"
    assert module.MOTION_EVIDENCE_ONLY == "MOTION_EVIDENCE_ONLY"
