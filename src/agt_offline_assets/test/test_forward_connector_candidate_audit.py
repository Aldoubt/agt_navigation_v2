import math

import numpy as np

from agt_offline_assets.agricultural_coverage_ordering import ConnectorRequest
from agt_offline_assets.forward_connector_candidate_audit import (
    ForwardConnectorCandidateAuditConfig,
    derive_forward_connector_candidate_audit,
    forward_connector_candidate_audit_to_dict,
)
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from agt_offline_assets.site_boundary import SiteBoundary
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile


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
        footprint_xy=(
            (0.42, 0.30),
            (0.42, -0.30),
            (-0.42, -0.30),
            (-0.42, 0.30),
        ),
        navigation_footprint_xy=(
            (0.42, 0.30),
            (0.42, -0.30),
            (-0.42, -0.30),
            (-0.42, 0.30),
        ),
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
    return TurnZoneSet(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        zones=(
            TurnZone(
                zone_id="turn_high_u",
                side="HIGH_U",
                polygon_xy=(
                    (-0.5, -0.5),
                    (0.5, -0.5),
                    (0.5, 2.5),
                    (-0.5, 2.5),
                ),
                supported_aisle_ids=("aisle_001", "aisle_002"),
                endpoint_count=2,
                free_fraction=float("nan"),
            ),
        ),
    )


def _grid(fill) -> NavigationGridEvidence:
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


def _boundary(x0=-0.20, x1=0.20) -> SiteBoundary:
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=(
            (x0, -1.0),
            (x1, -1.0),
            (x1, 3.0),
            (x0, 3.0),
        ),
        source={"fixture": "narrow_boundary"},
    )


def test_all_free_grid_classifies_forward_preview_free_and_orders_by_zone_extension():
    plan = derive_forward_connector_candidate_audit(
        (_request(),), _zones(), _grid(FREE), _vehicle()
    )
    result = plan.connectors[0]
    assert result.status == "FORWARD_PREVIEW_FREE"
    assert plan.forward_preview_free_count == 1
    assert result.candidate_count >= 4
    extensions = [
        item.max_required_zone_extension_m for item in result.candidates
    ]
    assert extensions == sorted(extensions)
    assert result.local_candidate_count >= 1


def test_occupied_grid_classifies_local_forward_as_known_blocked():
    plan = derive_forward_connector_candidate_audit(
        (_request(),), _zones(), _grid(OCCUPIED), _vehicle()
    )
    result = plan.connectors[0]
    assert result.status == "LOCAL_FORWARD_OCCUPANCY_BLOCKED"
    assert plan.local_occupancy_blocked_count == 1
    assert result.local_known_occupied_count == result.local_candidate_count
    assert all(
        item.footprint_evidence.occupied_fraction == 1.0
        for item in result.candidates
    )


def test_unknown_grid_classifies_local_map_evidence_insufficient():
    plan = derive_forward_connector_candidate_audit(
        (_request(),),
        _zones(),
        _grid(UNKNOWN),
        _vehicle(),
        ForwardConnectorCandidateAuditConfig(
            known_map_max_unknown_fraction=0.02
        ),
    )
    result = plan.connectors[0]
    assert result.status == "LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT"
    assert plan.local_map_insufficient_count == 1
    assert result.local_map_insufficient_count == result.local_candidate_count


def test_serialization_keeps_all_candidates_and_preview_only_boundary():
    plan = derive_forward_connector_candidate_audit(
        (_request(),), _zones(), _grid(OCCUPIED), _vehicle()
    )
    payload = forward_connector_candidate_audit_to_dict(plan)
    assert payload["schema"] == "agt_forward_connector_candidate_audit/v1"
    item = payload["connectors"][0]
    assert item["validation_scope"] == "PREVIEW_ONLY_NOT_R8_VEHICLE_READY"
    assert len(item["candidates"]) == item["candidate_count"]
    assert "required_zone_extension" in item["candidates"][0]
    assert "preview_footprint_evidence" in item["candidates"][0]


def test_candidate_audit_explicit_none_boundary_matches_default():
    requests = (_request(),)
    zones = _zones()
    navigation = _grid(FREE)
    vehicle = _vehicle()
    default = derive_forward_connector_candidate_audit(
        requests,
        zones,
        navigation,
        vehicle,
    )
    explicit = derive_forward_connector_candidate_audit(
        requests,
        zones,
        navigation,
        vehicle,
        site_boundary=None,
    )
    assert default == explicit


def test_candidate_audit_rejects_locally_relevant_boundary_crossing_set():
    plan = derive_forward_connector_candidate_audit(
        (_request(),),
        _zones(),
        _grid(FREE),
        _vehicle(),
        site_boundary=_boundary(),
    )
    assert plan.connectors[0].status == "LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT"
