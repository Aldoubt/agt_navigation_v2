import math

import numpy as np

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_safe_lane import derive_vehicle_safe_lane_plan


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
        blocked_reason="test preview only",
    )


def test_all_free_grid_does_not_override_structurally_too_narrow_aisle():
    points = tuple((float(x), 0.0, 0.0) for x in np.linspace(0.0, 3.0, 31))
    aisle = AislePrimitive(
        aisle_id="aisle_005",
        kind="interior",
        pair_kind="ROW_ROW",
        left_structure_ref="row_05",
        right_structure_ref="row_06",
        centerline_xyz=points,
        start_pose=(0.0, 0.0, 0.0, 0.0),
        end_pose=(3.0, 0.0, 0.0, 0.0),
        length_m=3.0,
        geometric_width_m=0.658,
        minimum_required_width_m=0.45,
        center_distance_m=1.298,
        longitudinal_overlap_m=3.0,
        safe_cell_count=100,
        centerline_cell_count=31,
        diagnostic_status="ACCEPTED",
    )
    graph = AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.8,
        aisles=(aisle,),
    )
    occupancy = np.full((80, 100), FREE, dtype=np.uint8)
    navigation = NavigationGridEvidence(
        resolution_m=0.05,
        origin_x_m=-1.0,
        origin_y_m=-2.0,
        width=100,
        height=80,
        occupancy=occupancy,
        frame_id="map",
    )

    plan = derive_vehicle_safe_lane_plan(graph, navigation, _vehicle())
    lane = plan.lanes[0]

    assert lane.status == "NO_VEHICLE_SAFE_LANE"
    assert "STRUCTURAL_WIDTH_BELOW_PREVIEW_VEHICLE_WIDTH" in lane.reason
    assert lane.centerline_xyz == ()
