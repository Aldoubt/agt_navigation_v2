from pathlib import Path

import numpy as np
import yaml

from agt_offline_assets.navigation_map_derivation import (
    FREE,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    write_navigation_map_freeze_bundle,
)


def _base_result() -> NavigationMapResult:
    shape = (2, 2)
    zeros_f = np.zeros(shape, dtype=np.float64)
    zeros_i = np.zeros(shape, dtype=np.int32)
    occupancy = np.full(shape, FREE, dtype=np.uint8)
    return NavigationMapResult(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=2,
        height=2,
        ground_height_m=zeros_f.copy(),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=zeros_i.copy(),
        ground_support_count=zeros_i.copy(),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=1.0),
    )


def _override():
    return {
        "id": "ovr_001",
        "mode": "force_occupied",
        "polygon_xy": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        "reason": "Measured permanent support post.",
        "evidence_category": "measured_structure",
    }


def test_freeze_bundle_writes_generated_and_accepted_maps(tmp_path: Path):
    output = write_navigation_map_freeze_bundle(
        _base_result(),
        tmp_path / "freeze",
        source_asset="processed.pcd",
        frame_id="map",
        overrides=[_override()],
    )

    assert (output / "generated" / "navigation_map.pgm").is_file()
    assert (output / "generated" / "navigation_map.yaml").is_file()
    assert (output / "accepted" / "navigation_map.pgm").is_file()
    assert (output / "accepted" / "navigation_map.yaml").is_file()

    record = yaml.safe_load((output / "derivation.yaml").read_text(encoding="utf-8"))
    assert record["source_asset"] == "processed.pcd"
    assert record["frame_id"] == "map"
    assert record["overrides"][0]["id"] == "ovr_001"
    assert (
        record["outputs"]["generated"]["pgm_sha256"]
        != record["outputs"]["accepted"]["pgm_sha256"]
    )


def test_freeze_bundle_without_overrides_preserves_identical_map_bytes(tmp_path: Path):
    output = write_navigation_map_freeze_bundle(
        _base_result(),
        tmp_path / "freeze",
        source_asset="processed.pcd",
        frame_id="map",
        overrides=[],
    )
    record = yaml.safe_load((output / "derivation.yaml").read_text(encoding="utf-8"))
    assert (
        record["outputs"]["generated"]["pgm_sha256"]
        == record["outputs"]["accepted"]["pgm_sha256"]
    )
    assert record["overrides"] == []


def test_freeze_bundle_is_byte_deterministic_for_same_inputs(tmp_path: Path):
    first = write_navigation_map_freeze_bundle(
        _base_result(),
        tmp_path / "first",
        source_asset="processed.pcd",
        frame_id="map",
        overrides=[_override()],
    )
    second = write_navigation_map_freeze_bundle(
        _base_result(),
        tmp_path / "second",
        source_asset="processed.pcd",
        frame_id="map",
        overrides=[_override()],
    )

    relative_files = (
        Path("generated/navigation_map.pgm"),
        Path("generated/navigation_map.yaml"),
        Path("accepted/navigation_map.pgm"),
        Path("accepted/navigation_map.yaml"),
        Path("derivation.yaml"),
    )
    for relative in relative_files:
        assert (first / relative).read_bytes() == (second / relative).read_bytes()
