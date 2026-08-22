import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PyQt5.QtWidgets import QApplication

from agt_map_workbench.review_diagnostics import derive_connectivity_breakpoint_mask
from agt_map_workbench.review_export import (
    REVIEW_LAYER_EXPORTS,
    review_layer_cloud_opacity,
    write_review_summary,
)
from agt_map_workbench.review_export_workbench import ReviewExportMapWorkbenchWindow


def test_review_layer_export_contract_covers_structure_and_formal_products():
    mapping = {key: filename for filename, key in REVIEW_LAYER_EXPORTS}
    assert mapping[None] == "01_pointcloud.png"
    assert mapping["final"] == "02_ground_relative.png"
    assert mapping["row_structural_band"] == "03_row_structural_band.png"
    assert mapping["aisle_geometric_envelope"] == "04_aisle_geometric_envelope.png"
    assert mapping["aisle_geometric_centerline"] == "05_aisle_geometric_centerline.png"
    assert mapping["aisle_centerline"] == "06_safe_aisle_centerline.png"
    assert mapping["formal_soft_occupied"] == "07_soft_occupied_candidate.png"
    assert mapping["formal_soft_recovered"] == "08_soft_occupied_recovered.png"
    assert mapping["formal_generated"] == "09_formal_generated.png"
    assert mapping["formal_accepted"] == "10_formal_accepted.png"
    assert mapping["formal_diff"] == "11_generated_accepted_diff.png"
    assert mapping["formal_hard_direct"] == "12_direct_obstacle.png"
    assert mapping["formal_hard_slope"] == "13_slope_hard.png"
    assert mapping["formal_hard_step"] == "14_step_hard.png"
    assert mapping["formal_hard_before_padding"] == "15_hard_before_padding.png"
    assert mapping["formal_hard_after_padding"] == "16_hard_after_padding.png"
    assert mapping["formal_aisle_hard_conflict"] == "17_aisle_hard_conflict.png"
    assert mapping["formal_connectivity_breakpoints"] == "18_connectivity_breakpoints.png"


def test_review_layer_cloud_opacity_keeps_pointcloud_context_without_hiding_formal_raster():
    assert review_layer_cloud_opacity(None) == 1.0
    assert review_layer_cloud_opacity("formal_generated") == 0.0
    assert review_layer_cloud_opacity("formal_accepted") == 0.0
    assert review_layer_cloud_opacity("formal_diff") == 0.0
    assert 0.0 < review_layer_cloud_opacity("formal_hard_slope") < 0.5
    assert 0.0 < review_layer_cloud_opacity("aisle_geometric_centerline") < 0.5


def test_connectivity_breakpoints_mark_cross_sections_with_no_free_cell():
    free = np.uint8(254)
    occupied = np.uint8(0)
    occupancy = np.full((3, 5), free, dtype=np.uint8)
    occupancy[:, 2] = occupied
    navigation = SimpleNamespace(
        occupancy=occupancy,
        origin_x_m=0.0,
        origin_y_m=0.0,
        resolution_m=1.0,
    )
    structure = SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.array([1.0, 0.0]))
    )
    diagnostic = SimpleNamespace(
        pair_index=0,
        pair_kind="ROW_ROW",
        status="ACCEPTED",
        left_row_center_v_m=0.0,
        right_row_center_v_m=3.0,
    )
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.ones((3, 5), dtype=bool),
        aisle_pair_diagnostics=(diagnostic,),
    )

    mask = derive_connectivity_breakpoint_mask(
        navigation,
        structure,
        corridor,
        occupancy,
    )

    expected = np.zeros((3, 5), dtype=bool)
    expected[:, 2] = True
    assert np.array_equal(mask, expected)


def test_write_review_summary_emits_json_and_human_readable_text(tmp_path: Path):
    summary = {
        "source": {"pcd": "/tmp/greenhouse.pcd", "points": 123},
        "agricultural_structure": {
            "accepted_rows": 21,
            "expected_interior_aisles": 20,
            "geometric_aisles": 20,
            "traversable_aisles": 5,
            "connected_aisles": 5,
        },
        "hard_occupancy_provenance": {
            "strong_sensor_obstacle": 100,
            "slope_hard": 20,
            "step_hard": 10,
            "padding_added_hard": 30,
        },
        "formal_navigation": {
            "structural_safety_status": "PASS",
            "navigation_usability_status": "REVIEW_REQUIRED",
            "largest_free_component_fraction": 0.626,
        },
    }

    json_path, text_path = write_review_summary(tmp_path, summary)

    assert json_path.name == "review_summary.json"
    assert text_path.name == "review_summary.txt"
    loaded = json.loads(json_path.read_text(encoding="utf-8"))
    assert loaded == summary
    text = text_path.read_text(encoding="utf-8")
    assert "geometric_aisles: 20" in text
    assert "connected_aisles: 5" in text
    assert "slope_hard: 20" in text
    assert "navigation_usability_status: REVIEW_REQUIRED" in text


def test_review_workbench_installs_geometric_export_and_provenance_controls():
    app = QApplication.instance() or QApplication([])
    window = ReviewExportMapWorkbenchWindow()
    try:
        geometric = window._find_layer_index("aisle_geometric_centerline")
        safe = window._find_layer_index("aisle_centerline")
        assert geometric >= 0
        assert safe >= 0
        assert "几何中心线" in window._nav_layer.itemText(geometric)
        assert "导航安全中心线" in window._nav_layer.itemText(safe)
        assert window._find_layer_index("formal_hard_direct") >= 0
        assert window._find_layer_index("formal_hard_slope") >= 0
        assert window._find_layer_index("formal_hard_step") >= 0
        assert window._find_layer_index("formal_hard_before_padding") >= 0
        assert window._find_layer_index("formal_hard_after_padding") >= 0
        assert window._find_layer_index("formal_aisle_hard_conflict") >= 0
        assert window._find_layer_index("formal_connectivity_breakpoints") >= 0
        assert window._review_export_button.text() == "一键导出地图审查包"
    finally:
        window.close()
        app.processEvents()
