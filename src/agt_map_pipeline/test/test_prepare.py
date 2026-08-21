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

def _canonical_assets(tmp_path: Path):
    alignment = tmp_path / "alignment.yaml"
    alignment.write_text("status: PASS\nmethod: TEST\nsource_frame: mapping_session\nmap_frame: map\ntransform:\n  yaw_rad: 0.0\n  translation_xyz_m: [0.0, 0.0, 0.0]\n")
    image = tmp_path / "accepted.pgm"
    width = height = 30
    image.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + bytes([205]) * width * height)
    map_yaml = tmp_path / "accepted.yaml"
    map_yaml.write_text("image: accepted.pgm\nresolution: 0.1\norigin: [-0.5, -0.5, 0.0]\n")
    return alignment, map_yaml

def test_prepare_finishes_at_waiting_human_review(tmp_path, greenhouse_pcd):
    result = prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=tmp_path / "project", site_id="synthetic")
    project = load_project(result.project_dir)
    assert result.project_state == "WAITING_HUMAN_REVIEW"
    assert tuple(project["stages"]) == ("source_profile", "navigation", "structure", "corridor", "aisle_graph", "traversability_preview")
    assert project["layers"]["navigation.raw_occupancy"]["status"] == "READY"
    assert project["layers"]["traversability.preview"]["status"] == "CANDIDATE_UNBOUNDED"
    assert project["frame"]["verification"] == "UNVERIFIED"

def test_resume_rejects_changed_source_sha(tmp_path, greenhouse_pcd):
    project = tmp_path / "project"
    prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=project)
    greenhouse_pcd.write_bytes(greenhouse_pcd.read_bytes() + b"changed")
    with pytest.raises(Exception, match="SHA"):
        prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=project, resume=True)

def test_prepare_requires_alignment_and_canonical_map_as_pair(tmp_path, greenhouse_pcd):
    alignment, map_yaml = _canonical_assets(tmp_path)
    with pytest.raises(ValueError, match="together"):
        prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=tmp_path / "a", alignment_path=alignment)
    with pytest.raises(ValueError, match="together"):
        prepare_project(greenhouse_pcd, preset_name="greenhouse", output_dir=tmp_path / "b", canonical_map_yaml=map_yaml)

def test_canonical_prepare_records_verified_frame_and_exact_grid(tmp_path, greenhouse_pcd):
    alignment, map_yaml = _canonical_assets(tmp_path)
    result = prepare_project(
        greenhouse_pcd,
        preset_name="greenhouse",
        output_dir=tmp_path / "project",
        site_id="synthetic",
        alignment_path=alignment,
        canonical_map_yaml=map_yaml,
    )
    project = load_project(result.project_dir)
    frame = project["frame"]
    assert frame["verification"] == "VERIFIED"
    assert frame["source_frame_id"] == "mapping_session"
    assert frame["canonical_frame_id"] == "map"
    assert frame["grid"]["origin_xy_m"] == [-0.5, -0.5]
    assert frame["grid"]["resolution_m"] == 0.1
    assert frame["grid"]["width"] == 30
    assert frame["grid"]["height"] == 30
    assert len(frame["alignment_sha256"]) == 64
    assert len(frame["grid"]["map_yaml_sha256"]) == 64
