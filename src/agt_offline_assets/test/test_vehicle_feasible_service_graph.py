import math

import pytest

from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
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


def _zone(
    zone_id="turn_low_u",
    side="LOW_U",
    aisle_ids=("aisle_001",),
    allow_turn=True,
):
    aisle_ids = tuple(aisle_ids)
    return TurnZone(
        zone_id=zone_id,
        side=side,
        polygon_xy=((-1.0, -2.0), (1.0, -2.0), (1.0, 2.0), (-1.0, 2.0)),
        supported_aisle_ids=aisle_ids,
        endpoint_count=len(aisle_ids),
        free_fraction=1.0,
        allow_turn=allow_turn,
    )


def _zones(*zones, frame_id="map", row_direction=(1.0, 0.0)):
    return TurnZoneSet(
        frame_id=frame_id,
        row_direction_xy=row_direction,
        zones=tuple(zones),
        source={},
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


def test_exactly_one_low_headland_adds_forward_in_reverse_out_candidate():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    segment = _segment(high_type=INTERIOR_BLOCKED_END)
    _resources, states = service_graph._build_service_resources_and_states(
        _plan(segment)
    )

    dead_end = [
        state
        for state in states
        if state.service_type == service_graph.DEAD_END_FORWARD_IN_REVERSE_OUT
    ]
    assert len(dead_end) == 1
    state = dead_end[0]
    assert state.entry_endpoint_type == LOW_U_HEADLAND
    assert state.exit_endpoint_type == LOW_U_HEADLAND
    assert state.entry_pose == segment.low_endpoint_pose
    assert state.exit_pose == segment.low_endpoint_pose
    assert state.forward_service_distance_m == pytest.approx(segment.length_m)
    assert state.reverse_service_distance_m == pytest.approx(segment.length_m)
    assert state.coverage_segment_id == segment.segment_id
    assert (
        state.validation_status
        == service_graph.REQUIRES_A3_REVERSE_SERVICE_VALIDATION
    )
    assert (
        state.external_reachability_status
        == service_graph.HEADLAND_TOPOLOGY_CANDIDATE
    )


def test_exactly_one_high_headland_reverses_dead_end_entry_heading():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    segment = _segment(
        low_type=INTERIOR_BLOCKED_END,
        high_type=HIGH_U_HEADLAND,
        yaw=0.25,
    )
    _resources, states = service_graph._build_service_resources_and_states(
        _plan(segment)
    )

    state = next(
        state
        for state in states
        if state.service_type == service_graph.DEAD_END_FORWARD_IN_REVERSE_OUT
    )
    expected_yaw = (0.25 + math.pi + math.pi) % (2.0 * math.pi) - math.pi
    assert state.entry_endpoint_type == HIGH_U_HEADLAND
    assert state.exit_endpoint_type == HIGH_U_HEADLAND
    assert state.entry_pose[:3] == segment.high_endpoint_pose[:3]
    assert state.entry_pose[3] == pytest.approx(expected_yaw)
    assert state.exit_pose == state.entry_pose


def test_double_interior_is_preserved_without_dead_end_candidate():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    segment = _segment(
        low_type=INTERIOR_BLOCKED_END,
        high_type=INTERIOR_BLOCKED_END,
    )
    resources, states = service_graph._build_service_resources_and_states(
        _plan(segment)
    )

    assert len(resources) == 1
    assert len(states) == 2
    assert all(
        state.service_type != service_graph.DEAD_END_FORWARD_IN_REVERSE_OUT
        for state in states
    )
    assert all(
        state.external_reachability_status
        == service_graph.EXTERNAL_REACHABILITY_UNPROVEN
        for state in states
    )


@pytest.mark.parametrize(
    ("plan_frame", "zone_frame"),
    (("map", "odom"), ("odom", "map")),
)
def test_validate_a2_inputs_rejects_frame_mismatch(plan_frame, zone_frame):
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    with pytest.raises(ValueError, match="frame"):
        service_graph._validate_a2_inputs(
            _plan(_segment(), frame_id=plan_frame),
            _zones(_zone(), frame_id=zone_frame),
        )


def test_validate_a2_inputs_rejects_opposite_row_direction():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    with pytest.raises(ValueError, match="row_direction"):
        service_graph._validate_a2_inputs(
            _plan(_segment(), row_direction=(1.0, 0.0)),
            _zones(_zone(), row_direction=(-1.0, 0.0)),
        )


def test_validate_a2_inputs_rejects_duplicate_zone_ids():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    duplicate = _zone()
    with pytest.raises(ValueError, match="duplicate.*zone"):
        service_graph._validate_a2_inputs(
            _plan(_segment()),
            _zones(duplicate, duplicate),
        )


def test_validate_a2_inputs_rejects_unknown_endpoint_type():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    with pytest.raises(ValueError, match="endpoint"):
        service_graph._validate_a2_inputs(
            _plan(_segment(low_type="UNKNOWN_ENDPOINT")),
            _zones(_zone()),
        )


def test_same_side_shared_turn_zone_emits_independent_directed_candidates():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    first = _segment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        y=0.0,
    )
    second = _segment(
        segment_id="aisle_002.segment_001",
        aisle_id="aisle_002",
        y=2.0,
    )
    _resources, states = service_graph._build_service_resources_and_states(
        _plan(first, second)
    )
    zone = _zone(aisle_ids=("aisle_001", "aisle_002"))

    candidates = service_graph._build_connector_candidates(states, _zones(zone))

    low_candidates = [candidate for candidate in candidates if candidate.side == "LOW_U"]
    assert len(low_candidates) == 2
    assert {
        (candidate.from_segment_id, candidate.to_segment_id)
        for candidate in low_candidates
    } == {
        ("aisle_001.segment_001", "aisle_002.segment_001"),
        ("aisle_002.segment_001", "aisle_001.segment_001"),
    }

    state_by_id = {state.service_state_id: state for state in states}
    for candidate in low_candidates:
        from_state = state_by_id[candidate.from_service_state_id]
        to_state = state_by_id[candidate.to_service_state_id]
        assert candidate.start_pose == from_state.exit_pose
        assert candidate.goal_pose == to_state.entry_pose
        assert (
            candidate.validation_status
            == service_graph.REQUIRES_A3_CONNECTOR_VALIDATION
        )


def test_connector_candidates_never_use_interior_ports():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    interior = _segment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        low_type=INTERIOR_BLOCKED_END,
        high_type=INTERIOR_BLOCKED_END,
    )
    through = _segment(
        segment_id="aisle_002.segment_001",
        aisle_id="aisle_002",
        y=2.0,
    )
    _resources, states = service_graph._build_service_resources_and_states(
        _plan(interior, through)
    )
    candidates = service_graph._build_connector_candidates(
        states,
        _zones(_zone(aisle_ids=("aisle_001", "aisle_002"))),
    )

    assert all(
        interior.segment_id not in (candidate.from_segment_id, candidate.to_segment_id)
        for candidate in candidates
    )


def test_connector_candidates_never_cross_low_and_high_domains():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    first = _segment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
    )
    second = _segment(
        segment_id="aisle_002.segment_001",
        aisle_id="aisle_002",
        y=2.0,
    )
    _resources, states = service_graph._build_service_resources_and_states(
        _plan(first, second)
    )
    candidates = service_graph._build_connector_candidates(
        states,
        _zones(
            _zone(
                zone_id="turn_low_u",
                side="LOW_U",
                aisle_ids=("aisle_001", "aisle_002"),
            ),
            _zone(
                zone_id="turn_high_u",
                side="HIGH_U",
                aisle_ids=("aisle_001", "aisle_002"),
            ),
        ),
    )

    state_by_id = {state.service_state_id: state for state in states}
    side_to_endpoint = {
        "LOW_U": LOW_U_HEADLAND,
        "HIGH_U": HIGH_U_HEADLAND,
    }
    assert candidates
    for candidate in candidates:
        from_state = state_by_id[candidate.from_service_state_id]
        to_state = state_by_id[candidate.to_service_state_id]
        assert from_state.exit_endpoint_type == side_to_endpoint[candidate.side]
        assert to_state.entry_endpoint_type == side_to_endpoint[candidate.side]


def test_connector_candidates_respect_allow_turn_and_supported_aisles():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    first = _segment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
    )
    second = _segment(
        segment_id="aisle_002.segment_001",
        aisle_id="aisle_002",
        y=2.0,
    )
    _resources, states = service_graph._build_service_resources_and_states(
        _plan(first, second)
    )

    disabled = service_graph._build_connector_candidates(
        states,
        _zones(
            _zone(
                aisle_ids=("aisle_001", "aisle_002"),
                allow_turn=False,
            )
        ),
    )
    unsupported = service_graph._build_connector_candidates(
        states,
        _zones(_zone(aisle_ids=("aisle_001",))),
    )

    assert disabled == ()
    assert unsupported == ()


def test_connector_candidates_forbid_same_physical_segment():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    segment = _segment()
    _resources, states = service_graph._build_service_resources_and_states(
        _plan(segment)
    )
    candidates = service_graph._build_connector_candidates(
        states,
        _zones(_zone(aisle_ids=(segment.aisle_id,))),
    )

    assert candidates == ()


def test_same_aisle_different_segment_is_not_hard_forbidden():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    first = _segment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        ordinal=1,
        length=3.0,
    )
    second = _segment(
        segment_id="aisle_001.segment_002",
        aisle_id="aisle_001",
        ordinal=2,
        length=3.0,
        y=0.2,
    )
    _resources, states = service_graph._build_service_resources_and_states(
        _plan(first, second)
    )
    candidates = service_graph._build_connector_candidates(
        states,
        _zones(_zone(aisle_ids=("aisle_001",))),
    )

    low_candidates = [candidate for candidate in candidates if candidate.side == "LOW_U"]
    assert {
        (candidate.from_segment_id, candidate.to_segment_id)
        for candidate in low_candidates
    } == {
        (first.segment_id, second.segment_id),
        (second.segment_id, first.segment_id),
    }


def test_same_resource_states_share_component_without_fake_motion_edge():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    segment = _segment(
        low_type=INTERIOR_BLOCKED_END,
        high_type=INTERIOR_BLOCKED_END,
    )
    resources, states = service_graph._build_service_resources_and_states(
        _plan(segment)
    )

    components = service_graph._build_candidate_components(resources, states, ())

    assert len(components) == 1
    component = components[0]
    assert component.classification == service_graph.CANDIDATE_TOPOLOGY_COMPONENT
    assert component.service_state_ids == tuple(
        sorted(state.service_state_id for state in states)
    )
    assert component.segment_ids == (segment.segment_id,)
    assert component.connector_candidate_ids == ()
    assert component.total_unique_coverage_length_m == pytest.approx(segment.length_m)


def test_connector_candidate_joins_resources_and_component_coverage_deduplicates_segment():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    first = _segment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        length=4.0,
    )
    second = _segment(
        segment_id="aisle_002.segment_001",
        aisle_id="aisle_002",
        length=6.0,
        y=2.0,
    )
    resources, states = service_graph._build_service_resources_and_states(
        _plan(first, second)
    )
    candidates = service_graph._build_connector_candidates(
        states,
        _zones(_zone(aisle_ids=("aisle_001", "aisle_002"))),
    )

    components = service_graph._build_candidate_components(
        resources,
        states,
        candidates,
    )

    assert len(components) == 1
    assert components[0].segment_ids == (
        "aisle_001.segment_001",
        "aisle_002.segment_001",
    )
    assert components[0].total_unique_coverage_length_m == pytest.approx(10.0)


def test_derive_service_graph_reports_static_candidate_diagnostics():
    from agt_offline_assets import vehicle_feasible_service_graph as service_graph

    first = _segment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        length=4.0,
    )
    second = _segment(
        segment_id="aisle_002.segment_001",
        aisle_id="aisle_002",
        length=4.0,
        y=2.0,
        high_type=INTERIOR_BLOCKED_END,
    )
    plan = _plan(first, second)
    zones = _zones(_zone(aisle_ids=("aisle_001", "aisle_002")))

    graph = service_graph.derive_vehicle_feasible_service_graph(
        plan,
        zones,
        source={
            "vehicle_feasible_segments_asset": "vehicle_feasible_segments.yaml",
            "turn_zones_asset": "turn_zones.yaml",
        },
    )

    assert graph.schema == service_graph.VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA
    assert graph.status == service_graph.TOPOLOGY_CANDIDATE_ONLY
    assert graph.frame_id == "map"
    assert graph.platform_id == "mk_mini"
    assert graph.diagnostics.resource_count == 2
    assert graph.diagnostics.ordinary_service_state_count == 4
    assert graph.diagnostics.dead_end_candidate_count == 1
    assert graph.diagnostics.total_service_state_count == 5
    assert graph.diagnostics.unique_coverage_length_m == pytest.approx(8.0)
    assert graph.source["derivation_kind"] == "V25_12G_A2_STATIC_SERVICE_TOPOLOGY"
    assert graph.source["vehicle_feasible_segment_schema"] == plan.schema
    assert graph.source["turn_zone_schema"] == zones.schema
