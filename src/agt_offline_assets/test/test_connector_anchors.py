import math

import numpy as np

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.agricultural_coverage_ordering import ConnectorRequest
from agt_offline_assets.connector_anchors import (
    ConnectorAnchorConfig,
    apply_connector_anchor_plan,
    derive_connector_anchor_plan,
)
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED
from agt_offline_assets.reverse_fallback_admission import (
    ReverseFallbackAdmissionItem,
    ReverseFallbackAdmissionPlan,
)
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile


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


def _aisle(aisle_id, y):
    points = tuple((float(x), float(y), 0.0) for x in np.linspace(0.0, 4.0, 41))
    return AislePrimitive(
        aisle_id=aisle_id,
        kind="interior",
        pair_kind="ROW_ROW",
        left_structure_ref="row_a",
        right_structure_ref="row_b",
        centerline_xyz=points,
        start_pose=(0.0, float(y), 0.0, 0.0),
        end_pose=(4.0, float(y), 0.0, 0.0),
        length_m=4.0,
        geometric_width_m=1.0,
        minimum_required_width_m=0.45,
        center_distance_m=1.6,
        longitudinal_overlap_m=4.0,
        safe_cell_count=100,
        centerline_cell_count=41,
        diagnostic_status="ACCEPTED",
    )


def _navigation():
    resolution = 0.05
    width = 140
    height = 100
    origin_x = -1.0
    origin_y = -1.5
    occupancy = np.full((height, width), FREE, dtype=np.uint8)
    # Obstacle strips beyond the raw x=4.0 aisle endpoints.  A centered
    # 0.84x0.60 footprint at x=4.0 overlaps them, while an inward-retreated
    # pose around x<=3.5 is FREE.
    for y in (0.0, 1.0):
        x0, x1 = 4.20, 4.45
        y0, y1 = y - 0.22, y + 0.22
        c0 = int((x0 - origin_x) / resolution)
        c1 = int((x1 - origin_x) / resolution)
        r0 = int((y0 - origin_y) / resolution)
        r1 = int((y1 - origin_y) / resolution)
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


def _admission():
    return ReverseFallbackAdmissionPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="abc",
        items=(
            ReverseFallbackAdmissionItem(
                connector_id="connector_001",
                from_aisle_id="aisle_001",
                to_aisle_id="aisle_002",
                turn_zone_id="turn_high_u",
                forward_audit_status="LOCAL_FORWARD_OCCUPANCY_BLOCKED",
                decision="ELIGIBLE_REVERSE_FALLBACK",
                reason="test",
            ),
        ),
    )


def test_raw_endpoint_can_be_retreated_to_stable_free_anchor():
    graph = AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.0,
        aisles=(_aisle("aisle_001", 0.0), _aisle("aisle_002", 1.0)),
    )
    request = ConnectorRequest(
        connector_id="connector_001",
        from_aisle_id="aisle_001",
        to_aisle_id="aisle_002",
        turn_zone_id="turn_high_u",
        side="HIGH_U",
        start_pose=(4.0, 0.0, 0.0, 0.0),
        goal_pose=(4.0, 1.0, 0.0, math.pi),
    )
    plan = derive_connector_anchor_plan(
        graph,
        (request,),
        _admission(),
        _navigation(),
        _vehicle(),
        ConnectorAnchorConfig(
            search_step_m=0.05,
            maximum_retreat_m=1.0,
            stable_free_span_m=0.20,
            preview_footprint_padding_m=0.05,
        ),
    )
    assert plan.ready_count == 1
    item = plan.items[0]
    assert item.start.raw_pose_free is False
    assert item.goal.raw_pose_free is False
    assert item.start.retreat_m is not None and item.start.retreat_m > 0.0
    assert item.goal.retreat_m is not None and item.goal.retreat_m > 0.0
    assert item.start.anchor_pose[0] < 4.0
    assert item.goal.anchor_pose[0] < 4.0

    applied = apply_connector_anchor_plan((request,), plan)
    assert applied[0].start_pose == item.start.anchor_pose
    assert applied[0].goal_pose == item.goal.anchor_pose


def test_only_r6a_eligible_connectors_are_anchored():
    graph = AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.0,
        aisles=(_aisle("aisle_001", 0.0), _aisle("aisle_002", 1.0)),
    )
    request = ConnectorRequest(
        connector_id="connector_999",
        from_aisle_id="aisle_001",
        to_aisle_id="aisle_002",
        turn_zone_id="turn_high_u",
        side="HIGH_U",
        start_pose=(4.0, 0.0, 0.0, 0.0),
        goal_pose=(4.0, 1.0, 0.0, math.pi),
    )
    plan = derive_connector_anchor_plan(
        graph,
        (request,),
        ReverseFallbackAdmissionPlan(
            frame_id="map",
            platform_id="mk_mini",
            platform_profile_sha256="abc",
            items=(),
        ),
        _navigation(),
        _vehicle(),
    )
    assert plan.items == ()
    assert plan.adjusted_requests == ()
