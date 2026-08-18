from agt_map_pipeline.prepare import prepare_project
from agt_map_pipeline.status import build_project_status
from agt_map_pipeline.project import load_project

def _pcd(path):
    pts = [(x*.1,y*.1,0.0) for x in range(20) for y in range(20)] + [(x*.1,y*.1,.3) for x in range(2,18) for y in (4,10,16)]
    path.write_text("\n".join(["VERSION 0.7", "FIELDS x y z", "SIZE 4 4 4", "TYPE F F F", "COUNT 1 1 1", f"WIDTH {len(pts)}", "HEIGHT 1", f"POINTS {len(pts)}", "DATA ascii"] + [f"{x} {y} {z}" for x,y,z in pts]) + "\n")

def test_identical_sources_produce_identical_layers(tmp_path):
    a, b = tmp_path / "a.pcd", tmp_path / "b.pcd"; _pcd(a); b.write_bytes(a.read_bytes())
    pa = tmp_path / "pa"; pb = tmp_path / "pb"
    prepare_project(a, preset_name="greenhouse", output_dir=pa); prepare_project(b, preset_name="greenhouse", output_dir=pb)
    da, db = load_project(pa), load_project(pb)
    assert build_project_status(pa)["project_state"] == "WAITING_HUMAN_REVIEW"
    assert da["preset"]["sha256"] == db["preset"]["sha256"]
    assert {k:v["sha256"] for k,v in da["layers"].items()} == {k:v["sha256"] for k,v in db["layers"].items()}
