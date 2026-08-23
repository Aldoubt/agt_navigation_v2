from pathlib import Path
import pytest

from agt_map_pipeline.project import (
    PROJECT_SCHEMA, create_project, load_project, register_layer,
    register_stage, write_project,
)


def test_create_project_records_unverified_frame_and_empty_review_requirements(tmp_path: Path):
    root = tmp_path / "project"
    doc = create_project(root, source={"absolute_path": "/data/site.pcd", "sha256": "a" * 64, "size_bytes": 123, "summary": {"point_count": 10}}, preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64}, declared_frame_id="map", site_id="greenhouse_01")
    assert doc["schema"] == PROJECT_SCHEMA
    assert doc["project_state"] == "NEW"
    assert doc["frame"]["verification"] == "UNVERIFIED"
    assert doc["map_authority"] is None
    assert doc["accepted_revision"] is None
    assert (root / "project.yaml").is_file()


def test_stage_and_layer_status_are_manifest_records_not_file_presence(tmp_path: Path):
    root = tmp_path / "project"
    doc = create_project(root, source={"absolute_path": "/x.pcd", "sha256": "a" * 64, "size_bytes": 1, "summary": {}}, preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64}, declared_frame_id="map", site_id=None)
    (root / "layers").mkdir(exist_ok=True)
    (root / "layers" / "fake.npy").write_bytes(b"exists")
    write_project(root, doc)
    assert "fake" not in load_project(root)["layers"]


def test_register_stage_rejects_unknown_status(tmp_path: Path):
    root = tmp_path / "project"
    doc = create_project(root, source={"absolute_path": "/x.pcd", "sha256": "a" * 64, "size_bytes": 1, "summary": {}}, preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64}, declared_frame_id="map", site_id=None)
    with pytest.raises(ValueError, match="status"):
        register_stage(doc, "pcd_profile", status="DONE", inputs_sha256="c" * 64, outputs=[], message="")


def test_project_rejects_invalid_map_authority_contract(tmp_path: Path):
    root = tmp_path / "project"
    doc = create_project(root, source={"absolute_path": "/x.pcd", "sha256": "a" * 64, "size_bytes": 1, "summary": {}}, preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64}, declared_frame_id="map", site_id=None)
    doc["map_authority"] = {
        "schema": "agt_v25_map_authority_binding/v1",
        "authority": "NOT_V25_WORKBENCH",
        "status": "BOUND_VERIFIED",
        "frame_id": "map",
        "grid": {"resolution_m": 0.05, "origin_xy_m": [0.0, 0.0], "width": 1, "height": 1},
    }
    with pytest.raises(ValueError, match="map authority"):
        write_project(root, doc)
