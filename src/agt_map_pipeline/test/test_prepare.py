from pathlib import Path
import pytest
from agt_map_pipeline.prepare import prepare_project
from agt_map_pipeline.project import load_project

@pytest.fixture
def greenhouse_pcd(tmp_path):
    p = tmp_path / "greenhouse.pcd"
    points = [(x * .1, y * .1, 0.0) for x in range(20) for y in range(20)] + [(x * .1, y * .1, .3) for x in range(2,18) for y in (4, 10, 16)]
    lines = ["# .PCD v0.7 - Point Cloud Data file format", "VERSION 0.7", "FIELDS x y z", "SIZE 4 4 4", "TYPE F F F", "COUNT 1 1 1", f"WIDTH {len(points)}", "HEIGHT 1", "VIEWPOINT 0 0 0 1 0 0 0", f"POINTS {len(points)}", "DATA ascii"] + [f"{x} {y} {z}" for x,y,z in points]
    p.write_text("\n".join(lines) + "\n")
    return p

def test_prepare_finishes_at_waiting_human_review(tmp_path, greenhouse_pcd):
    result = prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=tmp_path / "project", site_id="synthetic")
    project = load_project(result.project_dir)
    assert result.project_state == "WAITING_HUMAN_REVIEW"
    assert tuple(project["stages"]) == ("source_profile", "navigation", "structure", "corridor", "aisle_graph", "traversability_preview")
    assert project["layers"]["navigation.raw_occupancy"]["status"] == "READY"
    assert project["layers"]["traversability.preview"]["status"] == "CANDIDATE_UNBOUNDED"

def test_resume_rejects_changed_source_sha(tmp_path, greenhouse_pcd):
    project = tmp_path / "project"
    prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=project)
    greenhouse_pcd.write_bytes(greenhouse_pcd.read_bytes() + b"changed")
    with pytest.raises(Exception, match="SHA"):
        prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=project, resume=True)
