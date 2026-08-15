import math

import numpy as np

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE
from agt_offline_assets.vehicle_lane_feasibility import derive_vehicle_lane_feasibility_trace
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_safe_lane import (
    VehicleSafeLaneConfig,
    derive_vehicle_safe_lane_plan,
    vehicle_safe_lane_plan_to_dict,
)


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


def _aisle(width=1.60):
    points = tuple((float(x), 0.0, 0.0) for x in np.linspace(0.0, 4.0, 41))
    return AislePrimitive(
        aisle_id="aisle_001",
        kind="interior",
        pair_kind="ROW_ROW",
        left_structure_ref="row_01",
        right_structure_ref="row_02",
        centerline_xyz=points,
        start_pose=(0.0, 0.0, 0.0, 0.0),
        end_pose=(4.0, 0.0, 0.0, 0.0),
        length_m=4.0,
        geometric_width_m=float(width),
        minimum_required_width_m=0.45,
        center_distance_m=2.0,
        longitudinal_overlap_m=4.0,
        safe_cell_count=100,
        centerline_cell_count=41,
        diagnostic_status="ACCEPTED",
    )


def _graph(width=1.60):
    return AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=2.0,
        aisles=(_aisle(width),),
    )


def _navigation():
    resolution = 0.05
    origin_x = -1.0
    origin_y = -1.2
    width = 140
    height = 48
    occupancy = np.full((height, width), FREE, dtype=np.uint8)
    return NavigationGridEvidence(
        resolution_m=resolution,
        origin_x_m=origin_x,
        origin_y_m=origin_y,
        width=width,
        height=height,
        occupancy=occupancy,
        frame_id="map",
    )


def _config():
    return VehicleSafeLaneConfig(
        sample_spacing_m=0.10,
        lateral_search_step_m=0.05,
        maximum_lateral_shift_m=0.50,
        maximum_lateral_step_m=0.15,
        preview_footprint_padding_m=0.05,
        minimum_lane_coverage_fraction=0.70,
        maximum_endpoint_retreat_m=1.0,
        minimum_contiguous_span_m=1.0,
    )


def test_trace_preserves_every_sample_on_all_free_aisle():
    trace = derive_vehicle_lane_feasibility_trace(
        _aisle(),
        _navigation(),
        _vehicle(),
        _config(),
        row_direction_xy=(1.0, 0.0),
    )
    assert trace.aisle_id == "aisle_001"
    assert trace.structural_width_blocked_reason is None
    assert math.isclose(trace.structural_length_m, 4.0)
    assert len(trace.distances_m) == len(trace.selected_points)
    assert len(trace.selected_points) == len(trace.selected_offsets_m)
    assert all(point is not None for point in trace.selected_points)
    assert all(offset == 0.0 for offset in trace.selected_offsets_m)


def test_vehicle_safe_lane_serialization_contract_stays_stable():
    plan = derive_vehicle_safe_lane_plan(
        _graph(width=1.60),
        _navigation(),
        _vehicle(),
        _config(),
    )
    payload = vehicle_safe_lane_plan_to_dict(plan)
    assert payload["schema"] == "agt_vehicle_safe_aisle_lane/v1"
    assert payload["summary"] == {"ready": 1, "partial": 0, "unavailable": 0}
    assert payload["aisles"][0]["status"] == "VEHICLE_SAFE_LANE_READY"
    assert "segments" not in payload["aisles"][0]
