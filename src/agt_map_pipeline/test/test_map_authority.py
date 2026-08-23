from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path

import pytest


def _write_pgm(path: Path, width: int = 4, height: int = 3, value: int = 205) -> None:
    path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii")
        + bytes([value]) * width * height
    )


def make_v25_revision(tmp_path: Path) -> Path:
    revision = tmp_path / "revision_0001"
    generated = revision / "generated"
    accepted = revision / "accepted"
    generated.mkdir(parents=True)
    accepted.mkdir(parents=True)

    _write_pgm(generated / "navigation_map.pgm")
    _write_pgm(accepted / "navigation_map.pgm")
    for directory in (generated, accepted):
        (directory / "navigation_map.yaml").write_text(
            "image: navigation_map.pgm\n"
            "resolution: 0.05\n"
            "origin: [-2.5, -3.0, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )
    (revision / "derivation.yaml").write_text(
        "schema: agt_ground_relative_navigation_map/v1\nframe_id: map\n",
        encoding="utf-8",
    )
    return revision


def _authority_module():
    spec = importlib.util.find_spec("agt_map_pipeline.map_authority")
    assert spec is not None, "map_authority module must exist"
    return importlib.import_module("agt_map_pipeline.map_authority")


def test_bind_v25_revision_records_exact_hashes_and_grid(tmp_path: Path):
    module = _authority_module()
    revision = make_v25_revision(tmp_path)

    binding = module.bind_v25_map_revision(revision)

    assert binding["schema"] == "agt_v25_map_authority_binding/v1"
    assert binding["authority"] == "V25_MAP_WORKBENCH"
    assert binding["status"] == "BOUND_VERIFIED"
    assert binding["frame_id"] == "map"
    assert binding["grid"] == {
        "resolution_m": 0.05,
        "origin_xy_m": [-2.5, -3.0],
        "width": 4,
        "height": 3,
    }
    assert len(binding["generated_map_yaml_sha256"]) == 64
    assert len(binding["generated_map_pgm_sha256"]) == 64
    assert len(binding["accepted_map_yaml_sha256"]) == 64
    assert len(binding["accepted_map_pgm_sha256"]) == 64
    assert len(binding["derivation_sha256"]) == 64


def test_bind_v25_revision_rejects_incomplete_revision(tmp_path: Path):
    module = _authority_module()
    revision = make_v25_revision(tmp_path)
    (revision / "derivation.yaml").unlink()

    with pytest.raises(module.MapAuthorityError, match="derivation"):
        module.bind_v25_map_revision(revision)


def test_bound_authority_rejects_asset_drift(tmp_path: Path):
    module = _authority_module()
    revision = make_v25_revision(tmp_path)
    binding = module.bind_v25_map_revision(revision)
    Path(binding["accepted_map_yaml_path"]).write_text("changed\n", encoding="utf-8")

    with pytest.raises(module.MapAuthorityError, match="hash mismatch"):
        module.verify_bound_map_authority(binding)


def test_bound_authority_rejects_non_map_derivation(tmp_path: Path):
    module = _authority_module()
    revision = make_v25_revision(tmp_path)
    (revision / "derivation.yaml").write_text(
        "schema: agt_ground_relative_navigation_map/v1\nframe_id: odom\n",
        encoding="utf-8",
    )

    with pytest.raises(module.MapAuthorityError, match="frame"):
        module.bind_v25_map_revision(revision)
