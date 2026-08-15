import math

import numpy as np

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED
from agt_offline_assets.site_boundary import SiteBoundary
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_safe_lane import (
    VehicleSafeLaneConfig,
    derive_vehicle_safe_lane_plan,
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


def _navigation(*, center_obstacle=False):
    resolution = 0.05
    origin_x = -1.0
    origin_y = -1.2
    width = 140
    height = 48
    occupancy = np.full((height, width), FREE, dtype=np.uint8)
    if center_obstacle:
        y0, y1 = -0.05, 0.05
        x0, x1 = -0.5, 4.5
        c0 = int(math.floor((x0 - origin_x) / resolution))
        c1 = int(math.ceil((x1 - origin_x) / resolution))
        r0 = int(math.floor((y0 - origin_y) / resolution))
        r1 = int(math.ceil((y1 - origin_y) / resolution))
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


def test_all_free_structural_centerline_remains_vehicle_safe_lane():
    plan = derive_vehicle_safe_lane_plan(_graph(), _navigation(), _vehicle(), _config())
    assert plan.ready_count == 1
    lane = plan.lanes[0]
    assert lane.status == "VEHICLE_SAFE_LANE_READY"
    assert lane.coverage_fraction > 0.95
    assert lane.low_u_retreat_m == 0.0
    assert lane.high_u_retreat_m == 0.0
    assert lane.maximum_used_lateral_shift_m == 0.0
    assert lane.site_boundary_rejected_pose_count == 0
    assert lane.site_boundary_limited_sample_count == 0


def test_wide_aisle_can_shift_laterally_around_centerline_conflict():
    plan = derive_vehicle_safe_lane_plan(
        _graph(width=1.60),
        _navigation(center_obstacle=True),
        _vehicle(),
        _config(),
    )
    lane = plan.lanes[0]
    assert lane.status == "VEHICLE_SAFE_LANE_READY"
    assert lane.coverage_fraction > 0.95
    assert lane.maximum_used_lateral_shift_m >= 0.35
    assert all(abs(offset) >= 0.30 for offset in lane.lateral_offsets_m)


def test_narrow_aisle_does_not_force_vehicle_lane_through_occupied_cells():
    plan = derive_vehicle_safe_lane_plan(
        _graph(width=0.75),
        _navigation(center_obstacle=True),
        _vehicle(),
        _config(),
    )
    lane = plan.lanes[0]
    assert lane.status == "NO_VEHICLE_SAFE_LANE"
    assert lane.allowed_lateral_shift_m < 0.05
    assert lane.centerline_xyz == ()


def test_boundary_rejected_endpoint_poses_do_not_mean_whole_lane_is_blocked():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0)),
    )
    plan = derive_vehicle_safe_lane_plan(
        _graph(width=1.60),
        _navigation(),
        _vehicle(),
        _config(),
        site_boundary=boundary,
    )
    lane = plan.lanes[0]
    assert lane.status == "VEHICLE_SAFE_LANE_READY"
    assert lane.coverage_fraction >= 0.70
    assert lane.site_boundary_rejected_pose_count > 0
    assert lane.site_boundary_limited_sample_count > 0
    assert lane.site_boundary_limited_sample_count < lane.total_sample_count
    assert "SITE_BOUNDARY_CONFLICT" not in lane.reason


def test_all_free_grid_still_rejects_lane_when_footprint_crosses_site_boundary():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-1.0, -0.20), (5.0, -0.20), (5.0, 0.20), (-1.0, 0.20)),
    )
    plan = derive_vehicle_safe_lane_plan(
        _graph(width=1.60),
        _navigation(),
        _vehicle(),
        _config(),
        site_boundary=boundary,
    )
    lane = plan.lanes[0]
    assert lane.status == "NO_VEHICLE_SAFE_LANE"
    assert lane.centerline_xyz == ()
    assert lane.site_boundary_rejected_pose_count > 0
    assert lane.site_boundary_limited_sample_count == lane.total_sample_count
    assert "SITE_BOUNDARY_CONFLICT" in lane.reason
