from pathlib import Path

import yaml

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.agricultural_coverage_ordering import (
    CoverageOrderingConfig,
    derive_agricultural_coverage_order,
)
from agt_offline_assets.agricultural_route_io import (
    write_agricultural_aisle_graph,
    write_agricultural_coverage_order,
    write_turn_zones,
)
from agt_offline_assets.navigation_map_derivation import NAVIGATION_DERIVATION_SCHEMA
from agt_offline_assets.route_debug_dataset import (
    ASSET_INVALID,
    ASSET_LOADED,
    load_route_debug_dataset,
)
from agt_offline_assets.turn_zones import derive_turn_zone_candidates
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile


def _aisle(aisle_id: str, y: float) -> AislePrimitive:
    return AislePrimitive(
        aisle_id=aisle_id,
        kind="interior",
        pair_kind="ROW_ROW",
        left_structure_ref=f"{aisle_id}:left",
        right_structure_ref=f"{aisle_id}:right",
        centerline_xyz=((0.0, y, 0.0), (5.0, y, 0.0)),
        start_pose=(0.0, y, 0.0, 0.0),
        end_pose=(5.0, y, 0.0, 0.0),
        length_m=5.0,
        geometric_width_m=1.20,
        minimum_required_width_m=0.70,
        center_distance_m=1.80,
        longitudinal_overlap_m=5.0,
        safe_cell_count=50,
        centerline_cell_count=50,
        diagnostic_status="ACCEPTED",
    )


def _write_run(tmp_path: Path, *, coverage_frame: str = "map", include_no_go: bool = True):
    run_dir = tmp_path / "runtime" / "maps" / "debug_run"
    run_dir.mkdir(parents=True)
    profile_path = tmp_path / "profiles" / "platforms" / "preview_ackermann.yaml"
    profile_path.parent.mkdir(parents=True)
    profile_path.write_text(
        yaml.safe_dump(
            {
                "platform": {
                    "name": "preview_ackermann",
                    "kinematics": "ackermann",
                    "geometry": {
                        "length": 0.84,
                        "width": 0.60,
                        "footprint": [
                            [0.42, 0.30],
                            [0.42, -0.30],
                            [-0.42, -0.30],
                            [-0.42, 0.30],
                        ],
                        "min_turning_radius_verified": True,
                        "min_turning_radius": 1.5,
                    },
                    "route_acceptance": {
                        "enabled": True,
                        "preview_planning_enabled": True,
                    },
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    vehicle = load_canonical_vehicle_profile(profile_path)
    graph = AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.8,
        aisles=(_aisle("aisle_001", 1.0), _aisle("aisle_002", 2.8)),
        source={"fixture": True},
    )
    write_agricultural_aisle_graph(graph, run_dir / "aisle_graph.yaml")
    write_turn_zones(derive_turn_zone_candidates(graph), run_dir / "turn_zones.yaml")
    order = derive_agricultural_coverage_order(
        graph,
        vehicle,
        CoverageOrderingConfig(minimum_side_clearance_m=0.05),
    )
    coverage_path = write_agricultural_coverage_order(order, run_dir / "coverage_order.yaml")
    if coverage_frame != "map":
        payload = yaml.safe_load(coverage_path.read_text(encoding="utf-8"))
        payload["frame_id"] = coverage_frame
        coverage_path.write_text(
            yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
    if include_no_go:
        (run_dir / "derivation.yaml").write_text(
            yaml.safe_dump(
                {
                    "schema": NAVIGATION_DERIVATION_SCHEMA,
                    "frame_id": "map",
                    "overrides": [
                        {
                            "id": "operator_no_go_001",
                            "mode": "no_go",
                            "polygon_xy": [
                                [1.0, -0.5],
                                [2.0, -0.5],
                                [2.0, 0.5],
                                [1.0, 0.5],
                            ],
                        }
                    ],
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
    return run_dir, profile_path


def test_route_debug_dataset_joins_coverage_and_no_go(tmp_path: Path):
    run_dir, profile_path = _write_run(tmp_path)
    dataset = load_route_debug_dataset(run_dir, vehicle_profile_path=profile_path)

    assert dataset.frame_id == "map"
    assert dataset.asset_state("aisle_graph").availability == ASSET_LOADED
    assert dataset.asset_state("coverage_order").availability == ASSET_LOADED
    assert len(dataset.no_go_regions) == 1
    assert dataset.no_go_regions[0].polygon_xy[0] == (1.0, -0.5)
    assert len(dataset.aisles) == 2
    assert dataset.aisle_by_id("aisle_001") is not None
    assert dataset.aisle_by_id("aisle_001").traversal is not None
    assert dataset.aisle_by_id("aisle_001").traversal.motion_direction == "FORWARD"
    assert len(dataset.connector_requests) == 1
    assert len(dataset.connectors) == 1


def test_invalid_optional_coverage_frame_fails_closed_only_for_coverage(tmp_path: Path):
    run_dir, profile_path = _write_run(tmp_path, coverage_frame="odom", include_no_go=False)
    dataset = load_route_debug_dataset(run_dir, vehicle_profile_path=profile_path)

    assert dataset.frame_id == "map"
    assert dataset.asset_state("aisle_graph").availability == ASSET_LOADED
    assert dataset.asset_state("coverage_order").availability == ASSET_INVALID
    assert len(dataset.aisles) == 2
    assert dataset.connector_requests == ()
    assert dataset.connectors == ()
