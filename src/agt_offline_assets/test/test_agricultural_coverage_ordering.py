from pathlib import Path

import yaml

from agt_offline_assets import (
    AislePrimitive,
    AgriculturalAisleGraph,
    CoverageOrderingConfig,
    coverage_order_to_dict,
    derive_agricultural_coverage_order,
    load_canonical_vehicle_profile,
)


def _aisle(aisle_id: str, y: float, width: float, *, kind: str = "interior") -> AislePrimitive:
    return AislePrimitive(
        aisle_id=aisle_id,
        kind=kind,
        pair_kind="ROW_ROW" if kind == "interior" else "BOUNDARY_HIGH",
        left_structure_ref="row_01",
        right_structure_ref="row_02" if kind == "interior" else "boundary_high",
        centerline_xyz=((0.0, y, 0.0), (10.0, y, 0.0)),
        start_pose=(0.0, y, 0.0, 0.0),
        end_pose=(10.0, y, 0.0, 0.0),
        length_m=10.0,
        geometric_width_m=width,
        minimum_required_width_m=0.45,
        center_distance_m=1.0,
        longitudinal_overlap_m=10.0,
        safe_cell_count=100,
        centerline_cell_count=100,
        diagnostic_status="ACCEPTED",
    )


def _graph(widths=(1.0, 1.0, 1.0), *, last_kind="interior") -> AgriculturalAisleGraph:
    return AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.0,
        aisles=(
            _aisle("aisle_001", 0.0, widths[0]),
            _aisle("aisle_002", 1.0, widths[1]),
            _aisle("aisle_003", 2.0, widths[2], kind=last_kind),
        ),
    )


def _vehicle(tmp_path: Path):
    path = tmp_path / "mk_mini_like.yaml"
    payload = {
        "platform": {
            "name": "mk_mini_like",
            "kinematics": "ackermann",
            "geometry": {
                "length": 0.84,
                "width": 0.60,
                "wheel_base": 0.60,
                "track_width": 0.517,
                "wheel_diameter": 0.24,
                "ground_clearance": 0.111,
                "footprint": [[0.42, 0.30], [0.42, -0.30], [-0.42, -0.30], [-0.42, 0.30]],
                "navigation_footprint": [[0.42, 0.30], [0.42, -0.30], [-0.42, -0.30], [-0.42, 0.30]],
                "min_turning_radius_verified": True,
                "min_turning_radius": 1.5,
                "max_steering_angle_deg": 34.0,
            },
            "route_acceptance": {"enabled": True, "preview_planning_enabled": True},
        }
    }
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return load_canonical_vehicle_profile(path)


def test_boustrophedon_order_is_forward_motion_with_alternating_graph_orientation(tmp_path: Path):
    order = derive_agricultural_coverage_order(
        _graph(),
        _vehicle(tmp_path),
        CoverageOrderingConfig(minimum_side_clearance_m=0.05),
    )
    assert [item.aisle_id for item in order.traversals] == ["aisle_001", "aisle_002", "aisle_003"]
    assert [item.motion_direction for item in order.traversals] == ["FORWARD", "FORWARD", "FORWARD"]
    assert [item.graph_orientation for item in order.traversals] == [
        "WITH_ROW_DIRECTION",
        "AGAINST_ROW_DIRECTION",
        "WITH_ROW_DIRECTION",
    ]
    assert [item.entry_side for item in order.traversals] == ["LOW_U", "HIGH_U", "LOW_U"]
    assert [item.exit_side for item in order.traversals] == ["HIGH_U", "LOW_U", "HIGH_U"]
    assert [item.turn_zone_id for item in order.connector_requests] == ["turn_high_u", "turn_low_u"]
    assert order.traversals[1].entry_pose[3] != 0.0


def test_vehicle_width_filter_rejects_too_narrow_aisle(tmp_path: Path):
    order = derive_agricultural_coverage_order(
        _graph(widths=(1.0, 0.65, 1.0)),
        _vehicle(tmp_path),
        CoverageOrderingConfig(minimum_side_clearance_m=0.05),
    )
    assert [item.aisle_id for item in order.traversals] == ["aisle_001", "aisle_003"]
    assert len(order.rejected_aisles) == 1
    rejection = order.rejected_aisles[0]
    assert rejection.aisle_id == "aisle_002"
    assert rejection.reason == "VEHICLE_WIDTH_INFEASIBLE"
    assert rejection.required_width_m == 0.70


def test_boundary_aisle_can_be_disabled_without_renumbering(tmp_path: Path):
    order = derive_agricultural_coverage_order(
        _graph(last_kind="boundary"),
        _vehicle(tmp_path),
        CoverageOrderingConfig(include_boundary_aisles=False),
    )
    assert [item.aisle_id for item in order.traversals] == ["aisle_001", "aisle_002"]
    assert order.rejected_aisles[0].aisle_id == "aisle_003"
    assert order.rejected_aisles[0].reason == "BOUNDARY_AISLE_DISABLED_BY_POLICY"
    payload = coverage_order_to_dict(order)
    assert payload["schema"] == "agt_agricultural_coverage_order/v1"
    assert payload["traversal_count"] == 2
    assert payload["connector_request_count"] == 1
