from pathlib import Path

from agt_map_pipeline.prepare import prepare_project
from agt_map_pipeline.status import build_project_status
from agt_map_pipeline.project import load_project

def _pcd(path):
    pts = [(x*.1,y*.1,0.0) for x in range(20) for y in range(20)] + [(x*.1,y*.1,.3) for x in range(2,18) for y in (4,10,16)]
    path.write_text("\n".join(["VERSION 0.7", "FIELDS x y z", "SIZE 4 4 4", "TYPE F F F", "COUNT 1 1 1", f"WIDTH {len(pts)}", "HEIGHT 1", f"POINTS {len(pts)}", "DATA ascii"] + [f"{x} {y} {z}" for x,y,z in pts]) + "\n")

def _v25_revision(root: Path):
    revision = root / "v25_revision"
    for name in ("generated", "accepted"):
        directory = revision / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "navigation_map.pgm").write_bytes(b"P5\n30 30\n255\n" + bytes([205]) * 900)
        (directory / "navigation_map.yaml").write_text("image: navigation_map.pgm\nresolution: 0.1\norigin: [-0.5, -0.5, 0.0]\n")
    (revision / "derivation.yaml").write_text("schema: agt_ground_relative_navigation_map/v1\nframe_id: map\n")
    return revision

def test_identical_sources_produce_identical_layers(tmp_path):
    a, b = tmp_path / "a.pcd", tmp_path / "b.pcd"; _pcd(a); b.write_bytes(a.read_bytes())
    pa = tmp_path / "pa"; pb = tmp_path / "pb"
    prepare_project(a, preset_name="greenhouse", output_dir=pa); prepare_project(b, preset_name="greenhouse", output_dir=pb)
    da, db = load_project(pa), load_project(pb)
    assert build_project_status(pa)["project_state"] == "WAITING_HUMAN_REVIEW"
    assert da["preset"]["sha256"] == db["preset"]["sha256"]
    assert {k:v["sha256"] for k,v in da["layers"].items()} == {k:v["sha256"] for k,v in db["layers"].items()}

def test_formal_mode_is_deterministic_without_becoming_map_authority(tmp_path):
    source = tmp_path / "source.pcd"; _pcd(source)
    revision = _v25_revision(tmp_path)
    pa = tmp_path / "formal_a"; pb = tmp_path / "formal_b"
    prepare_project(source, preset_name="greenhouse", output_dir=pa, v25_map_revision=revision)
    prepare_project(source, preset_name="greenhouse", output_dir=pb, v25_map_revision=revision)
    da, db = load_project(pa), load_project(pb)
    assert da["map_authority"]["accepted_map_yaml_sha256"] == db["map_authority"]["accepted_map_yaml_sha256"]
    assert da["layers"]["evidence.navigation_occupancy"]["sha256"] == db["layers"]["evidence.navigation_occupancy"]["sha256"]
    assert "navigation.nav2_yaml" not in da["layers"]
    assert build_project_status(pa)["map_authority"]["authority"] == "V25_MAP_WORKBENCH"
