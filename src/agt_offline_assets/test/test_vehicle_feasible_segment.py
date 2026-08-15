from dataclasses import replace
import math

import numpy as np
import pytest

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.forward_connector import ForwardConnectorSample
from agt_offline_assets.forward_connector_navigation_gate import (
    _preview_local_footprint,
    _transform_polygon,
)
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED
from agt_offline_assets.site_boundary import (
    SiteBoundary,
    polygon_strictly_inside_site_boundary,
)
from agt_offline_assets.vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    derive_vehicle_feasible_segment_plan,
    load_vehicle_feasible_segment_plan,
    vehicle_feasible_segment_plan_to_dict,
    write_vehicle_feasible_segment_plan,
)
from agt_offline_assets.vehicle_lane_feasibility import (
    derive_vehicle_lane_feasibility_trace,
)
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_safe_lane import VehicleSafeLaneConfig


def _vehicle():
    footprint = ((0.42, 0.30), (0.42, -0.30), (-0.42, -0.30), (-0.42, 0.30))
    return CanonicalVehicleProfile(
        profile_id="mk_mini",
        profile_path="mk_mini.yaml",
        profile_sha256="abc",
        kinematics="ackermann",
        footprint_frame="base_footprint",
        base_frame="base_link",
        physical_length_m=0.84,
        physical_width_m=0.60,
        footprint_xy=footprint,
        navigation_footprint_xy=footprint,
        navigation_width_m=0.60,
        navigation_length_m=0.84,
        wheel_base_m=0.60,
        track_width_m=0.517,
        wheel_diameter_m=0.24,
        ground_clearance_m=0.111,
        minimum_turning_radius_m=1.5,
        minimum_turning_radius_verified=True,
        maximum_steering_angle_rad=math.radians(34.0),
        maximum_steering_angle_deg=34.0,
        allow_in_place_rotation=False,
        max_forward_velocity_mps=1.0,
        max_reverse_velocity_mps=0.5,
        max_angular_velocity_rps=None,
        manufacturer_maximum_speed_mps=None,
        route_acceptance_enabled=False,
        preview_planning_enabled=True,
        blocked_reason="physical base_footprint reference not yet measured",
    )


def _aisle(length_m=5.0, width=1.60):
    sample_count = int(round(length_m / 0.10)) + 1
    points = tuple(
        (float(x), 0.0, 0.0)
        for x in np.linspace(0.0, float(length_m), sample_count)
    )
    return AislePrimitive(
        aisle_id="aisle_001",
        kind="interior",
        pair_kind="ROW_ROW",
        left_structure_ref="row_01",
        right_structure_ref="row_02",
        centerline_xyz=points,
        start_pose=(0.0, 0.0, 0.0, 0.0),
        end_pose=(float(length_m), 0.0, 0.0, 0.0),
        length_m=float(length_m),
        geometric_width_m=float(width),
        minimum_required_width_m=0.45,
        center_distance_m=2.0,
        longitudinal_overlap_m=float(length_m),
        safe_cell_count=100,
        centerline_cell_count=sample_count,
        diagnostic_status="ACCEPTED",
    )


def _graph(length_m=5.0, width=1.60):
    return AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=2.0,
        aisles=(_aisle(length_m=length_m, width=width),),
    )


def _navigation_with_blocked_x_ranges(ranges):
    resolution = 0.05
    origin_x = -1.0
    origin_y = -1.5
    width = 180
    height = 60
    occupancy = np.full((height, width), FREE, dtype=np.uint8)

    # Block the complete lateral search band plus the padded MK-mini footprint,
    # so a blocked longitudinal interval cannot be bypassed by lateral shifting.
    y0, y1 = -1.0, 1.0
    r0 = int(math.floor((y0 - origin_y) / resolution))
    r1 = int(math.ceil((y1 - origin_y) / resolution))
    for x0, x1 in ranges:
        c0 = int(math.floor((float(x0) - origin_x) / resolution))
        c1 = int(math.ceil((float(x1) - origin_x) / resolution))
        occupancy[r0 : r1 + 1, c0 : c1 + 1] = OCCUPIED

    return NavigationGridEvidence(
        resolution_m=resolution,
        origin_x_m=origin_x,
        origin_y_m=origin_y,
        width=width,
        height=height,
        occupancy=occupancy,
        frame_id="map",
    )


def _navigation_with_centerline_strip():
    navigation = _navigation_with_blocked_x_ranges(())
    occupancy = navigation.occupancy.copy()
    resolution = float(navigation.resolution_m)
    y0, y1 = -0.05, 0.05
    x0, x1 = -0.5, 5.5
    r0 = int(math.floor((y0 - navigation.origin_y_m) / resolution))
    r1 = int(math.ceil((y1 - navigation.origin_y_m) / resolution))
    c0 = int(math.floor((x0 - navigation.origin_x_m) / resolution))
    c1 = int(math.ceil((x1 - navigation.origin_x_m) / resolution))
    occupancy[r0 : r1 + 1, c0 : c1 + 1] = OCCUPIED
    return replace(navigation, occupancy=occupancy)


def _config(*, minimum_contiguous_span_m=1.0):
    return VehicleSafeLaneConfig(
        sample_spacing_m=0.10,
        lateral_search_step_m=0.05,
        maximum_lateral_shift_m=0.50,
        maximum_lateral_step_m=0.15,
        preview_footprint_padding_m=0.05,
        minimum_lane_coverage_fraction=0.70,
        maximum_endpoint_retreat_m=1.0,
        minimum_contiguous_span_m=float(minimum_contiguous_span_m),
    )


def _large_boundary():
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-2.0, -2.0), (7.0, -2.0), (7.0, 2.0), (-2.0, 2.0)),
    )


def test_two_disjoint_useful_runs_are_both_emitted():
    plan = derive_vehicle_feasible_segment_plan(
        _graph(length_m=5.0),
        _navigation_with_blocked_x_ranges(((2.20, 2.80),)),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    aisle = plan.aisles[0]
    assert len(aisle.active_segments) == 2
    assert [segment.segment_id for segment in aisle.active_segments] == [
        "aisle_001.segment_001",
        "aisle_001.segment_002",
    ]
    assert aisle.active_segments[0].end_distance_m < aisle.active_segments[1].start_distance_m
    assert all(segment.length_m >= 1.0 for segment in aisle.active_segments)


def test_short_feasible_run_is_preserved_only_as_rejected_fragment():
    plan = derive_vehicle_feasible_segment_plan(
        _graph(length_m=6.0),
        _navigation_with_blocked_x_ranges(((2.00, 2.20), (3.60, 3.80))),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    aisle = plan.aisles[0]

    assert len(aisle.active_segments) == 2
    assert len(aisle.rejected_fragments) == 1
    fragment = aisle.rejected_fragments[0]
    assert fragment.fragment_id == "aisle_001.fragment_001"
    assert fragment.length_m < 1.0
    assert fragment.reason == "BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH"


def test_only_outer_active_segment_endpoints_are_headland_candidates():
    plan = derive_vehicle_feasible_segment_plan(
        _graph(length_m=5.0),
        _navigation_with_blocked_x_ranges(((2.20, 2.80),)),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    first, second = plan.aisles[0].active_segments

    assert first.low_endpoint_type == LOW_U_HEADLAND
    assert first.high_endpoint_type == INTERIOR_BLOCKED_END
    assert second.low_endpoint_type == INTERIOR_BLOCKED_END
    assert second.high_endpoint_type == HIGH_U_HEADLAND


def test_frame_mismatch_fails_closed_before_segment_generation():
    navigation = replace(_navigation_with_blocked_x_ranges(()), frame_id="odom")

    with pytest.raises(ValueError, match="frame_id"):
        derive_vehicle_feasible_segment_plan(
            _graph(length_m=5.0),
            navigation,
            _vehicle(),
            _config(minimum_contiguous_span_m=1.0),
        )


def test_preview_not_ready_vehicle_fails_closed():
    vehicle = replace(_vehicle(), preview_planning_enabled=False)
    with pytest.raises(ValueError, match="not ready for planning preview"):
        derive_vehicle_feasible_segment_plan(
            _graph(),
            _navigation_with_blocked_x_ranges(()),
            vehicle,
            _config(),
        )


def test_invalid_site_boundary_fails_closed():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)),
    )
    with pytest.raises(ValueError, match="site_boundary"):
        derive_vehicle_feasible_segment_plan(
            _graph(),
            _navigation_with_blocked_x_ranges(()),
            _vehicle(),
            _config(),
            site_boundary=boundary,
        )


def test_nonfinite_aisle_geometry_fails_closed():
    aisle = replace(
        _aisle(),
        centerline_xyz=((0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0)),
    )
    graph = replace(_graph(), aisles=(aisle,))
    with pytest.raises(ValueError, match="non-finite"):
        derive_vehicle_feasible_segment_plan(
            graph,
            _navigation_with_blocked_x_ranges(()),
            _vehicle(),
            _config(),
        )


def test_structural_width_gate_emits_zero_segments_with_reason():
    plan = derive_vehicle_feasible_segment_plan(
        _graph(width=0.65),
        _navigation_with_blocked_x_ranges(()),
        _vehicle(),
        _config(),
    )
    aisle = plan.aisles[0]
    assert aisle.active_segments == ()
    assert aisle.rejected_fragments == ()
    assert aisle.raw_feasible_fragment_count == 0
    assert aisle.allowed_lateral_shift_m == 0.0
    assert aisle.reason == (
        "STRUCTURAL_WIDTH_BELOW_PREVIEW_VEHICLE_WIDTH: "
        "aisle=0.650 m required=0.700 m"
    )


def test_site_boundary_clipping_keeps_every_emitted_pose_strictly_inside():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.50, -1.0), (4.50, -1.0), (4.50, 1.0), (0.50, 1.0)),
    )
    vehicle = _vehicle()
    config = _config()
    plan = derive_vehicle_feasible_segment_plan(
        _graph(),
        _navigation_with_blocked_x_ranges(()),
        vehicle,
        config,
        site_boundary=boundary,
    )
    aisle = plan.aisles[0]
    assert aisle.active_segments
    assert aisle.site_boundary_rejected_pose_count > 0

    local_footprint = _preview_local_footprint(
        vehicle,
        config.preview_footprint_padding_m,
    )
    for segment in aisle.active_segments:
        for point in segment.centerline_xyz:
            sample = ForwardConnectorSample(
                x=float(point[0]),
                y=float(point[1]),
                z=float(point[2]),
                yaw=0.0,
            )
            polygon = _transform_polygon(local_footprint, sample)
            assert polygon_strictly_inside_site_boundary(boundary, polygon)


def test_site_boundary_can_fully_block_aisle():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-1.0, -0.20), (6.0, -0.20), (6.0, 0.20), (-1.0, 0.20)),
    )
    plan = derive_vehicle_feasible_segment_plan(
        _graph(),
        _navigation_with_blocked_x_ranges(()),
        _vehicle(),
        _config(),
        site_boundary=boundary,
    )
    aisle = plan.aisles[0]
    assert aisle.active_segments == ()
    assert aisle.raw_feasible_fragment_count == 0
    assert aisle.site_boundary_rejected_pose_count > 0
    assert aisle.site_boundary_limited_sample_count > 0


def test_all_navigation_poses_blocked_emits_zero_segments():
    plan = derive_vehicle_feasible_segment_plan(
        _graph(),
        _navigation_with_blocked_x_ranges(((-0.50, 5.50),)),
        _vehicle(),
        _config(),
    )
    aisle = plan.aisles[0]
    assert aisle.active_segments == ()
    assert aisle.rejected_fragments == ()
    assert aisle.raw_feasible_fragment_count == 0
    assert aisle.grid_rejected_pose_count > 0


def test_lateral_shift_segment_preserves_shared_trace_selection():
    graph = _graph()
    navigation = _navigation_with_centerline_strip()
    vehicle = _vehicle()
    config = _config()
    trace = derive_vehicle_lane_feasibility_trace(
        graph.aisles[0],
        navigation,
        vehicle,
        config,
        row_direction_xy=graph.row_direction_xy,
    )
    plan = derive_vehicle_feasible_segment_plan(
        graph,
        navigation,
        vehicle,
        config,
    )
    segment = plan.aisles[0].active_segments[0]
    expected_offsets = tuple(
        float(value) for value in trace.selected_offsets_m if value is not None
    )
    assert segment.lateral_offsets_m == expected_offsets
    assert segment.maximum_used_lateral_shift_m >= 0.30


def test_repeated_derivation_is_deterministic():
    args = (
        _graph(length_m=5.0),
        _navigation_with_blocked_x_ranges(((2.20, 2.80),)),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    first = derive_vehicle_feasible_segment_plan(*args, site_boundary=_large_boundary())
    second = derive_vehicle_feasible_segment_plan(*args, site_boundary=_large_boundary())
    assert first == second
    assert [segment.segment_id for segment in first.aisles[0].active_segments] == [
        "aisle_001.segment_001",
        "aisle_001.segment_002",
    ]


def test_segment_yaml_round_trip_is_deterministic(tmp_path):
    plan = derive_vehicle_feasible_segment_plan(
        _graph(length_m=5.0),
        _navigation_with_blocked_x_ranges(((2.20, 2.80),)),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    payload = vehicle_feasible_segment_plan_to_dict(plan)
    assert tuple(payload) == (
        "schema",
        "status",
        "frame_id",
        "platform_id",
        "platform_profile_sha256",
        "row_direction_xy",
        "source",
        "configuration",
        "summary",
        "aisles",
    )

    first = write_vehicle_feasible_segment_plan(plan, tmp_path / "first.yaml")
    second = write_vehicle_feasible_segment_plan(plan, tmp_path / "second.yaml")
    assert first.read_bytes() == second.read_bytes()
    assert load_vehicle_feasible_segment_plan(first) == plan


def test_segment_yaml_writer_refuses_implicit_overwrite(tmp_path):
    plan = derive_vehicle_feasible_segment_plan(
        _graph(length_m=5.0),
        _navigation_with_blocked_x_ranges(((2.20, 2.80),)),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    output = tmp_path / "vehicle_feasible_segments.yaml"
    write_vehicle_feasible_segment_plan(plan, output)

    with pytest.raises(FileExistsError):
        write_vehicle_feasible_segment_plan(plan, output)

    rewritten = write_vehicle_feasible_segment_plan(plan, output, overwrite=True)
    assert rewritten == output.resolve()
