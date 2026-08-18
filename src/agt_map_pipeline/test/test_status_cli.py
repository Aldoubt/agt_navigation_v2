import json
import pytest
from agt_map_pipeline.prepare import prepare_project
from agt_map_pipeline.status import build_project_status
from agt_map_pipeline.cli import main

@pytest.fixture
def greenhouse_pcd(tmp_path):
    p = tmp_path / "greenhouse.pcd"
    pts = [(x*.1,y*.1,0.0) for x in range(20) for y in range(20)] + [(x*.1,y*.1,.3) for x in range(2,18) for y in (4,10,16)]
    p.write_text("\n".join(["VERSION 0.7", "FIELDS x y z", "SIZE 4 4 4", "TYPE F F F", "COUNT 1 1 1", f"WIDTH {len(pts)}", "HEIGHT 1", f"POINTS {len(pts)}", "DATA ascii"] + [f"{x} {y} {z}" for x,y,z in pts]) + "\n")
    return p

def test_status_json_is_agent_oriented(tmp_path, greenhouse_pcd, capsys):
    project = tmp_path / "project"; prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=project)
    status = build_project_status(project)
    assert status["schema"] == "agt_map_project_status/v1"
    assert status["project_state"] == "WAITING_HUMAN_REVIEW"
    assert status["layers"]["navigation.raw_occupancy"] == "READY"
    assert status["layers"]["traversability.preview"] == "CANDIDATE_UNBOUNDED"
    assert status["next_action"]["human_required"] is True

def test_status_json_exits_zero(tmp_path, greenhouse_pcd, capsys):
    project = tmp_path / "project"; prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=project)
    assert main(["status", str(project), "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["project_state"] == "WAITING_HUMAN_REVIEW"

def test_prepare_returns_two(tmp_path, greenhouse_pcd):
    assert main(["prepare", str(greenhouse_pcd), "--preset", "greenhouse", "--output", str(tmp_path / "project")]) == 2
