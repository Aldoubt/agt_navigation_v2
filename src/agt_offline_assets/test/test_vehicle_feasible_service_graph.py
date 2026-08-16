import math

import pytest

from agt_offline_assets.vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    LOW_U_HEADLAND,
    AisleFeasibleSegmentResult,
    VehicleFeasibleSegment,
    VehicleFeasibleSegmentPlan,
)


_A1_SOURCE = {
    "vehicle_safe_lane_configuration": {
        "sample_spacing_m": 0.10,
        "lateral_search_step_m": 0.05,
        "maximum_lateral_shift_m": 0.50,
        "maximum_lateral_step_m": 0.15,
        "preview_footprint_padding_m": 0.05,
        "minimum_lane_coverage_fraction": 0.70,
        "maximum_endpoint_retreat_m": 2.00,
        "minimum_contiguous_span_m": 1.00,
    }
}


def _segment(
    segment_id="aisle_001.segment_001",
    aisle_id="aisle_001",
    ordinal=1,
    *,
    length=4.0,
    y=0.0,
    low_type=LOW_U_HEADLAND,
    high_type=HIGH_U_HEADLAND,
    yaw=0.0,
):
    return VehicleFeasibleSegment(
        segment_id=segment_id,
        aisle_id=aisle_id,
        ordinal_in_aisle=ordinal,
        start_distance_m=0.0,
        end_distance_m=length,
        length_m=length,
        coverage_fraction_of_aisle=1.0,
        low_endpoint_type=low_type,
        high_endpoint_type=high_type,
        centerline_xyz=((0.0, y, 0.0), (length, y, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(0.0, y, 0.0, yaw),
        high_endpoint_pose=(length, y, 0.0, yaw),
    )


def _plan(*segments, frame_id="map", row_direction=(1.0, 0.0)):
    grouped = {}
    for segment in segments:
        grouped.setdefault(segment.aisle_id, []).append(segment)

    aisles = []
    for aisle_id in sorted(grouped):
        items = tuple(
            sorted(grouped[aisle_id], key=lambda item: item.ordinal_in_aisle)
        )
        aisles.append(
            AisleFeasibleSegmentResult(
                aisle_id=aisle_id,
                structural_length_m=sum(item.length_m for item in items),
                active_segments=items,
                rejected_fragments=(),
                raw_feasible_fragment_count=len(items),
                allowed_lateral_shift_m=0.5,
                site_boundary_rejected_pose_count=0,
                site_boundary_limited_sample_count=0,
                grid_rejected_pose_count=0,
                reason="test fixture",
            )
        )

    return VehicleFeasibleSegmentPlan(
        frame_id=frame_id,
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        row_direction_xy=row_direction,
        aisles=tuple(aisles),
        source=dict(_A1_SOURCE),
    )


def test_resource_expands_into_two_forward_directional_states():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    resources, states = service_graph._build_service_resources_and_states(
        _plan(_segment())
    )

    assert len(resources) == 1
    assert resources[0].segment_id == "aisle_001.segment_001"
    assert resources[0].coverage_length_m == pytest.approx(4.0)
    assert len(states) == 2

    low_to_high = next(
        state
        for state in states
        if state.service_type == service_graph.SERVICE_LOW_TO_HIGH
    )
    high_to_low = next(
        state
        for state in states
        if state.service_type == service_graph.SERVICE_HIGH_TO_LOW
    )

    assert low_to_high.service_state_id == (
        "aisle_001.segment_001.service_low_to_high"
    )
    assert low_to_high.entry_endpoint_type == LOW_U_HEADLAND
    assert low_to_high.exit_endpoint_type == HIGH_U_HEADLAND
    assert low_to_high.entry_pose == (0.0, 0.0, 0.0, 0.0)
    assert low_to_high.exit_pose == (4.0, 0.0, 0.0, 0.0)
    assert low_to_high.service_motion_direction == service_graph.FORWARD
    assert low_to_high.forward_service_distance_m == pytest.approx(4.0)
    assert low_to_high.reverse_service_distance_m == pytest.approx(0.0)
    assert low_to_high.coverage_segment_id == resources[0].segment_id
    assert low_to_high.coverage_reward_length_m == pytest.approx(4.0)
    assert low_to_high.validation_status == service_graph.TOPOLOGY_SERVICE_CANDIDATE
    assert (
        low_to_high.external_reachability_status
        == service_graph.HEADLAND_TOPOLOGY_CANDIDATE
    )

    assert high_to_low.service_state_id == (
        "aisle_001.segment_001.service_high_to_low"
    )
    assert high_to_low.entry_endpoint_type == HIGH_U_HEADLAND
    assert high_to_low.exit_endpoint_type == LOW_U_HEADLAND
    assert high_to_low.entry_pose[:3] == (4.0, 0.0, 0.0)
    assert high_to_low.exit_pose[:3] == (0.0, 0.0, 0.0)
    assert high_to_low.entry_pose[3] == pytest.approx(-math.pi)
    assert high_to_low.exit_pose[3] == pytest.approx(-math.pi)
    assert high_to_low.service_motion_direction == service_graph.FORWARD
