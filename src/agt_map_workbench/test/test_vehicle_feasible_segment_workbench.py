import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication

from agt_offline_assets.vehicle_feasible_segment import (
    AisleFeasibleSegmentResult,
    RejectedFeasibleFragment,
    VehicleFeasibleSegment,
    VehicleFeasibleSegmentPlan,
    write_vehicle_feasible_segment_plan,
)
from agt_map_workbench.agricultural_workbench import AgriculturalMapWorkbenchWindow
from agt_map_workbench.review_workbench import ReviewMapWorkbenchWindow


def qapp():
    return QApplication.instance() or QApplication([])


def _valid_plan() -> VehicleFeasibleSegmentPlan:
    segment = VehicleFeasibleSegment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        ordinal_in_aisle=1,
        start_distance_m=0.0,
        end_distance_m=1.5,
        length_m=1.5,
        coverage_fraction_of_aisle=0.5,
        low_endpoint_type="LOW_U_HEADLAND",
        high_endpoint_type="INTERIOR_BLOCKED_END",
        centerline_xyz=((0.0, 1.0, 0.0), (1.5, 1.5, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(0.0, 1.0, 0.0, 0.0),
        high_endpoint_pose=(1.5, 1.5, 0.0, 0.0),
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
        structural_length_m=3.0,
        active_segments=(segment,),
        rejected_fragments=(fragment,),
        raw_feasible_fragment_count=2,
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
        source={
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
        },
    )


def test_missing_vehicle_feasible_segment_sibling_is_quiet_and_clears_state(tmp_path):
    app = qapp()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    source = run_dir / "map.pcd"
    source.write_text("placeholder\n", encoding="utf-8")

    window = ReviewMapWorkbenchWindow()
    try:
        window._source_path = source
        window._load_vehicle_feasible_segment_sibling()

        assert window._vehicle_feasible_segment_plan is None
        assert window._vehicle_feasible_segment_last_error == ""
        assert window._vehicle_feasible_segment_preview.active_item_count == 0
        app.processEvents()
    finally:
        window.close()


def test_valid_vehicle_feasible_segment_sibling_loads_plan_and_preview(tmp_path):
    app = qapp()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    source = run_dir / "map.pcd"
    source.write_text("placeholder\n", encoding="utf-8")
    expected = _valid_plan()
    write_vehicle_feasible_segment_plan(
        expected,
        run_dir / "vehicle_feasible_segments.yaml",
    )

    window = ReviewMapWorkbenchWindow()
    try:
        window._source_path = source
        window._load_vehicle_feasible_segment_sibling()

        assert window._vehicle_feasible_segment_plan == expected
        assert window._vehicle_feasible_segment_last_error == ""
        assert window._vehicle_feasible_segment_preview.active_item_count == 1
        assert window._vehicle_feasible_segment_preview.rejected_fragment_count == 1
        assert (
            window._vehicle_feasible_segment_preview.rejected_fragment_total_length_m
            == pytest.approx(0.5)
        )
        assert not any(
            item.isVisible()
            for item in window._scene.items()
            if item.zValue() == pytest.approx(12.0)
        )
        app.processEvents()
    finally:
        window.close()


def test_invalid_vehicle_feasible_segment_sibling_is_local_and_legacy_dispatch_survives(
    tmp_path,
):
    app = qapp()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    source = run_dir / "map.pcd"
    source.write_text("placeholder\n", encoding="utf-8")
    (run_dir / "vehicle_feasible_segments.yaml").write_text(
        "schema: not-an-a1-segment-plan\nstatus: DRAFT\n",
        encoding="utf-8",
    )

    window = ReviewMapWorkbenchWindow()
    try:
        window._source_path = source
        window._load_vehicle_feasible_segment_sibling()

        assert window._vehicle_feasible_segment_plan is None
        assert window._vehicle_feasible_segment_last_error
        assert window._vehicle_feasible_segment_preview.active_item_count == 0
        assert window._vehicle_feasible_segment_preview.rejected_fragment_count == 0

        window._nav_layer.setCurrentIndex(0)
        window._update_navigation_overlay()
        app.processEvents()
    finally:
        window.close()


def test_source_change_clears_stale_vehicle_feasible_segment_state(monkeypatch, tmp_path):
    app = qapp()
    first_dir = tmp_path / "first"
    second_dir = tmp_path / "second"
    first_dir.mkdir()
    second_dir.mkdir()
    first_source = first_dir / "map.pcd"
    second_source = second_dir / "map.pcd"
    first_source.write_text("placeholder\n", encoding="utf-8")
    second_source.write_text("placeholder\n", encoding="utf-8")
    write_vehicle_feasible_segment_plan(
        _valid_plan(),
        first_dir / "vehicle_feasible_segments.yaml",
    )

    window = ReviewMapWorkbenchWindow()
    try:
        window._source_path = first_source
        window._load_vehicle_feasible_segment_sibling()
        assert window._vehicle_feasible_segment_plan is not None
        assert window._vehicle_feasible_segment_preview.active_item_count == 1

        def _fake_base_open_pcd(self):
            self._source_path = second_source

        monkeypatch.setattr(
            AgriculturalMapWorkbenchWindow,
            "_open_pcd",
            _fake_base_open_pcd,
        )
        window._open_pcd()

        assert window._source_path == second_source
        assert window._vehicle_feasible_segment_plan is None
        assert window._vehicle_feasible_segment_last_error == ""
        assert window._vehicle_feasible_segment_preview.active_item_count == 0
        assert window._vehicle_feasible_segment_preview.rejected_fragment_count == 0
        app.processEvents()
    finally:
        window.close()


def test_vehicle_feasible_segment_layer_selected_with_overlay_enabled_shows_preview(
    tmp_path,
):
    app = qapp()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    source = run_dir / "map.pcd"
    source.write_text("placeholder\n", encoding="utf-8")
    write_vehicle_feasible_segment_plan(
        _valid_plan(),
        run_dir / "vehicle_feasible_segments.yaml",
    )

    window = ReviewMapWorkbenchWindow()
    try:
        window._source_path = source
        window._load_vehicle_feasible_segment_sibling()
        assert window._vehicle_feasible_segment_preview.active_item_count == 1

        layer_index = next(
            (
                index
                for index in range(window._nav_layer.count())
                if str(window._nav_layer.itemData(index)) == "vehicle_feasible_segments"
            ),
            -1,
        )
        assert layer_index >= 0
        assert window._nav_layer.itemText(layer_index) == "12G-A1 Vehicle-Feasible Segments"

        window._nav_overlay_visible.setChecked(True)
        window._nav_layer.setCurrentIndex(layer_index)
        window._update_navigation_overlay()

        a1_items = [
            item
            for item in window._scene.items()
            if item.zValue() == pytest.approx(12.0)
        ]
        assert len(a1_items) == 1
        assert all(item.isVisible() for item in a1_items)
        assert not window._navigation_preview_item.isVisible()
        app.processEvents()
    finally:
        window.close()


def test_switching_away_from_vehicle_feasible_segment_layer_hides_preview(tmp_path):
    app = qapp()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    source = run_dir / "map.pcd"
    source.write_text("placeholder\n", encoding="utf-8")
    write_vehicle_feasible_segment_plan(
        _valid_plan(),
        run_dir / "vehicle_feasible_segments.yaml",
    )

    window = ReviewMapWorkbenchWindow()
    try:
        window._source_path = source
        window._load_vehicle_feasible_segment_sibling()

        a1_index = next(
            index
            for index in range(window._nav_layer.count())
            if str(window._nav_layer.itemData(index)) == "vehicle_feasible_segments"
        )
        other_index = next(
            index
            for index in range(window._nav_layer.count())
            if str(window._nav_layer.itemData(index)) != "vehicle_feasible_segments"
        )

        window._nav_overlay_visible.setChecked(True)
        window._nav_layer.setCurrentIndex(a1_index)
        window._update_navigation_overlay()
        a1_items = [
            item
            for item in window._scene.items()
            if item.zValue() == pytest.approx(12.0)
        ]
        assert len(a1_items) == 1
        assert all(item.isVisible() for item in a1_items)

        window._nav_layer.setCurrentIndex(other_index)
        window._update_navigation_overlay()

        assert not any(item.isVisible() for item in a1_items)
        app.processEvents()
    finally:
        window.close()
