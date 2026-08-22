import json
from pathlib import Path

from agt_map_workbench.review_export import (
    REVIEW_LAYER_EXPORTS,
    write_review_summary,
)


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
    assert "navigation_usability_status: REVIEW_REQUIRED" in text
