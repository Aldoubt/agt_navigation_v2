import math

import numpy as np

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED
from agt_offline_assets.vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    derive_vehicle_feasible_segment_plan,
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
