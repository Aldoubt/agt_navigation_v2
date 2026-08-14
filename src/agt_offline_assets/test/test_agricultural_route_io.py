from pathlib import Path

import yaml

from agt_offline_assets import (
    AislePrimitive,
    AgriculturalAisleGraph,
    CoverageOrderingConfig,
    derive_agricultural_coverage_order,
    derive_forward_connector_plan,
    derive_turn_zone_candidates,
    load_agricultural_aisle_graph,
    load_canonical_vehicle_profile,
    load_coverage_connector_requests,
    load_turn_zones,
    write_agricultural_aisle_graph,
    write_agricultural_coverage_order,
    write_forward_connector_plan,
    write_turn_zones,
)


def test_exported_aisle_graph_can_be_reloaded_without_workbench(tmp_path: Path):
    graph = AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.8,
        aisles=(
            AislePrimitive(
                aisle_id="aisle_001",
                kind="interior",
                pair_kind="ROW_ROW",
                left_structure_ref="row_01",
                right_structure_ref="row_02",
                centerline_xyz=((0.0, 0.0, -1.0), (5.0, 0.0, -0.9)),
                start_pose=(0.0, 0.0, -1.0, 0.0),
                end_pose=(5.0, 0.0, -0.9, 0.0),
                length_m=5.0,
                geometric_width_m=1.0,
                minimum_required_width_m=0.45,
                center_distance_m=1.8,
                longitudinal_overlap_m=5.0,
                safe_cell_count=50,
                centerline_cell_count=50,
                diagnostic_status="ACCEPTED",
            ),
            AislePrimitive(
                aisle_id="aisle_002",
                kind="boundary",
                pair_kind="BOUNDARY_HIGH",
                left_structure_ref="row_02",
                right_structure_ref="boundary_high",
                centerline_xyz=((0.0, 1.8, -1.0), (5.0, 1.8, -0.9)),
                start_pose=(0.0, 1.8, -1.0, 0.0),
                end_pose=(5.0, 1.8, -0.9, 0.0),
                length_m=5.0,
                geometric_width_m=1.2,
                minimum_required_width_m=0.45,
                center_distance_m=1.8,
                longitudinal_overlap_m=5.0,
                safe_cell_count=60,
                centerline_cell_count=50,
                diagnostic_status="ACCEPTED",
            ),
        ),
        source={"pcd_name": "processed.pcd"},
    )
    aisle_path = write_agricultural_aisle_graph(graph, tmp_path / "aisle_graph.yaml")
    loaded = load_agricultural_aisle_graph(aisle_path)
    assert loaded == graph

    vehicle_path = tmp_path / "vehicle.yaml"
    vehicle_path.write_text(
        yaml.safe_dump(
            {
                "platform": {
                    "name": "preview_ackermann",
                    "kinematics": "ackermann",
                    "geometry": {
                        "length": 0.84,
                        "width": 0.60,
                        "footprint": [[0.42, 0.30], [0.42, -0.30], [-0.42, -0.30], [-0.42, 0.30]],
                        "min_turning_radius_verified": True,
                        "min_turning_radius": 1.5,
                    },
                    "route_acceptance": {"enabled": True, "preview_planning_enabled": True},
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    vehicle = load_canonical_vehicle_profile(vehicle_path)
    order = derive_agricultural_coverage_order(
        loaded,
        vehicle,
        CoverageOrderingConfig(minimum_side_clearance_m=0.05),
    )
    coverage_path = write_agricultural_coverage_order(order, tmp_path / "coverage_order.yaml")
    payload = yaml.safe_load(coverage_path.read_text(encoding="utf-8"))
    assert payload["schema"] == "agt_agricultural_coverage_order/v1"
    assert payload["traversal_count"] == 2
    assert payload["connector_request_count"] == 1

    frozen_requests = load_coverage_connector_requests(coverage_path)
    assert frozen_requests == order.connector_requests

    zones = derive_turn_zone_candidates(loaded)
    zone_path = write_turn_zones(zones, tmp_path / "turn_zones.yaml")
    frozen_zones = load_turn_zones(zone_path)
    assert frozen_zones.frame_id == zones.frame_id
    assert frozen_zones.row_direction_xy == zones.row_direction_xy
    assert tuple(zone.zone_id for zone in frozen_zones.zones) == tuple(zone.zone_id for zone in zones.zones)

    plan = derive_forward_connector_plan(frozen_requests, frozen_zones, vehicle)
    plan_path = write_forward_connector_plan(plan, tmp_path / "forward_connectors.yaml")
    plan_payload = yaml.safe_load(plan_path.read_text(encoding="utf-8"))
    assert plan_payload["schema"] == "agt_forward_connector_plan/v1"
    assert plan_payload["connector_count"] == 1
    assert plan_payload["connectors"][0]["validation_scope"] == "CENTERLINE_KINEMATICS_AND_TURN_ZONE_ONLY"
