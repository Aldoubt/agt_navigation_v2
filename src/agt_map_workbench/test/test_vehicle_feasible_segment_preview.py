import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QGraphicsItem, QGraphicsScene

from agt_offline_assets.vehicle_feasible_segment import (
    AisleFeasibleSegmentResult,
    RejectedFeasibleFragment,
    VehicleFeasibleSegment,
    VehicleFeasibleSegmentPlan,
)
from agt_map_workbench.vehicle_feasible_segment_preview import (
    VehicleFeasibleSegmentPreview,
)


def _plan() -> VehicleFeasibleSegmentPlan:
    first = VehicleFeasibleSegment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        ordinal_in_aisle=1,
        start_distance_m=0.0,
        end_distance_m=1.5,
        length_m=1.5,
        coverage_fraction_of_aisle=0.30,
        low_endpoint_type="LOW_U_HEADLAND",
        high_endpoint_type="INTERIOR_BLOCKED_END",
        centerline_xyz=((0.0, 1.0, 0.0), (1.5, 1.5, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(0.0, 1.0, 0.0, 0.0),
        high_endpoint_pose=(1.5, 1.5, 0.0, 0.0),
    )
    second = VehicleFeasibleSegment(
        segment_id="aisle_001.segment_002",
        aisle_id="aisle_001",
        ordinal_in_aisle=2,
        start_distance_m=3.0,
        end_distance_m=5.0,
        length_m=2.0,
        coverage_fraction_of_aisle=0.40,
        low_endpoint_type="INTERIOR_BLOCKED_END",
        high_endpoint_type="HIGH_U_HEADLAND",
        centerline_xyz=((3.0, -0.5, 0.0), (5.0, -1.0, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(3.0, -0.5, 0.0, 0.0),
        high_endpoint_pose=(5.0, -1.0, 0.0, 0.0),
    )
    fragment = RejectedFeasibleFragment(
        fragment_id="aisle_001.fragment_001",
        aisle_id="aisle_001",
        start_distance_m=2.0,
        end_distance_m=2.5,
        length_m=0.5,
    )
    aisle = AisleFeasibleSegmentResult(
        aisle_id="aisle_001",
        structural_length_m=5.0,
        active_segments=(first, second),
        rejected_fragments=(fragment,),
        raw_feasible_fragment_count=3,
        allowed_lateral_shift_m=0.5,
        site_boundary_rejected_pose_count=0,
        site_boundary_limited_sample_count=0,
        grid_rejected_pose_count=1,
        reason="fixture",
    )
    return VehicleFeasibleSegmentPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="abc",
        row_direction_xy=(1.0, 0.0),
        aisles=(aisle,),
    )


def test_preview_renders_only_explicit_active_geometry_and_keeps_fragment_diagnostics():
    app = QApplication.instance() or QApplication([])
    scene = QGraphicsScene()
    preview = VehicleFeasibleSegmentPreview(scene)

    preview.set_plan(_plan())

    assert preview.active_item_count == 2
    assert preview.rejected_fragment_count == 1
    assert preview.rejected_fragment_total_length_m == pytest.approx(0.5)
    assert len(scene.items()) == 2

    for item in scene.items():
        assert not bool(item.flags() & QGraphicsItem.ItemIsMovable)
        assert not bool(item.flags() & QGraphicsItem.ItemIsSelectable)
        assert item.zValue() == pytest.approx(12.0)

    first_path = next(
        item.path()
        for item in scene.items()
        if item.path().elementCount() >= 2
        and item.path().elementAt(0).x == pytest.approx(0.0)
    )
    assert first_path.elementAt(0).y == pytest.approx(-1.0)
    assert first_path.elementAt(1).y == pytest.approx(-1.5)

    preview.set_visible(False)
    assert not any(item.isVisible() for item in scene.items())

    preview.clear()
    assert preview.active_item_count == 0
    assert preview.rejected_fragment_count == 0
    assert preview.rejected_fragment_total_length_m == pytest.approx(0.0)
    assert scene.items() == []

    app.processEvents()
