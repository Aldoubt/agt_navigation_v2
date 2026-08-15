import math

import numpy as np

from agt_offline_assets import (
    CanonicalVehicleProfile,
    ConnectorRequest,
    ForwardConnectorNavigationGateConfig,
    NavigationGridEvidence,
    SiteBoundary,
    TurnZone,
    TurnZoneSet,
    derive_forward_connector_navigation_gate,
    forward_connector_navigation_gate_to_dict,
)
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED


def _vehicle() -> CanonicalVehicleProfile:
    return CanonicalVehicleProfile(
        profile_id="mk_mini",
        profile_path="profiles/platforms/mk_mini.yaml",
        profile_sha256="deadbeef",
        kinematics="ackermann",
        footprint_frame="base_footprint",
        base_frame="base_link",
        physical_length_m=0.84,
        physical_width_m=0.60,
        footprint_xy=((0.42, 0.30), (0.42, -0.30), (-0.42, -0.30), (-0.42, 0.30)),
        navigation_footprint_xy=((0.42, 0.30), (0.42, -0.30), (-0.42, -0.30), (-0.42, 0.30)),
        navigation_width_m=0.60,
        navigation_length_m=0.84,
        wheel_base_m=0.60,
        track_width_m=0.517,
        wheel_diameter_m=0.240,
        ground_clearance_m=0.111,
        minimum_turning_radius_m=1.50,
        minimum_turning_radius_verified=True,
        maximum_steering_angle_rad=math.radians(34.0),
        maximum_steering_angle_deg=34.0,
        allow_in_place_rotation=False,
        max_forward_velocity_mps=1.0,
        max_reverse_velocity_mps=0.5,
        max_angular_velocity_rps=None,
        manufacturer_maximum_speed_mps=9.7 / 3.6,
        route_acceptance_enabled=False,
        preview_planning_enabled=True,
        blocked_reason="preview-only fixture",
    )


def _request() -> ConnectorRequest:
    return ConnectorRequest(
        connector_id="connector_001",
        from_aisle_id="aisle_001",
        to_aisle_id="aisle_002",
        turn_zone_id="turn_high_u",
        side="HIGH_U",
        start_pose=(0.0, 0.0, -1.6, 0.0),
        goal_pose=(0.0, 2.0, -1.5, math.pi),
    )


def _zones() -> TurnZoneSet:
    zone = TurnZone(
        zone_id="turn_high_u",
        side="HIGH_U",
        polygon_xy=((-0.5, -0.5), (0.5, -0.5), (0.5, 2.5), (-0.5, 2.5)),
        supported_aisle_ids=("aisle_001", "aisle_002"),
        endpoint_count=2,
        free_fraction=float("nan"),
    )
    return TurnZoneSet(frame_id="map", row_direction_xy=(1.0, 0.0), zones=(zone,))


def _grid(fill=FREE) -> NavigationGridEvidence:
    occupancy = np.full((120, 120), fill, dtype=np.uint8)
    return NavigationGridEvidence(
        resolution_m=0.10,
        origin_x_m=-6.0,
        origin_y_m=-5.0,
        width=120,
        height=120,
        occupancy=occupancy,
        source={"fixture": "navigation_grid"},
    )


def test_all_free_navigation_grid_accepts_one_forward_preview_candidate():
    plan = derive_forward_connector_navigation_gate(
        (_request(),),
        _zones(),
        _grid(),
        _vehicle(),
        ForwardConnectorNavigationGateConfig(
            sample_step_m=0.05,
            preview_footprint_padding_m=0.05,
        ),
    )
    assert plan.preview_free_count == 1
    result = plan.connectors[0]
    assert result.status == "PREVIEW_FOOTPRINT_FREE"
    assert result.path_type in {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"}
    assert result.footprint_evidence.occupied_count == 0
    assert result.footprint_evidence.unknown_count == 0
    assert result.footprint_evidence.grid_coverage_fraction == 1.0
    assert result.max_required_zone_extension_m >= 0.0


def test_fully_occupied_navigation_grid_rejects_all_forward_preview_candidates():
    plan = derive_forward_connector_navigation_gate(
        (_request(),),
        _zones(),
        _grid(OCCUPIED),
        _vehicle(),
    )
    assert plan.preview_free_count == 0
    assert plan.review_required_count == 1
    result = plan.connectors[0]
    assert result.status == "NO_FORWARD_PREVIEW_FREE_CANDIDATE"
    assert result.footprint_evidence.occupied_fraction == 1.0
    assert len(result.samples) > 2


def test_free_grid_candidate_is_rejected_when_swept_footprint_crosses_boundary():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-0.10, -0.20), (0.10, -0.20), (0.10, 2.20), (-0.10, 2.20)),
    )
    plan = derive_forward_connector_navigation_gate(
        (_request(),),
        _zones(),
        _grid(),
        _vehicle(),
        site_boundary=boundary,
    )
    assert plan.preview_free_count == 0
    result = plan.connectors[0]
    assert result.status == "NO_FORWARD_PREVIEW_FREE_CANDIDATE"
    assert "SITE_BOUNDARY_CONFLICT" in result.reason


def test_serialization_preserves_preview_only_boundary_and_path_evidence():
    plan = derive_forward_connector_navigation_gate(
        (_request(),),
        _zones(),
        _grid(),
        _vehicle(),
    )
    payload = forward_connector_navigation_gate_to_dict(plan)
    assert payload["schema"] == "agt_forward_connector_navigation_gate/v1"
    assert payload["status"] == "DRAFT"
    item = payload["connectors"][0]
    assert item["validation_scope"] == "PREVIEW_ONLY_NOT_R8_VEHICLE_READY"
    assert "preview_footprint_evidence" in item
    assert "required_zone_extension" in item
    assert item["samples"][0]["motion_direction"] == "FORWARD"
