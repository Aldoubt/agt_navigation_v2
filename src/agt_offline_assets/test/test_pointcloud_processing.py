from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_offline_assets import (
    AssetContractError,
    PcdCloud,
    PcdSchema,
    process_pointcloud,
    read_pcd,
    sha256_file,
    validate_pointcloud_processing,
    write_pcd,
)


def _cloud(points):
    schema = PcdSchema(
        fields=("x", "y", "z", "intensity"),
        sizes=(4, 4, 4, 4),
        types=("F", "F", "F", "F"),
        counts=(1, 1, 1, 1),
        viewpoint=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    )
    data = np.empty(len(points), dtype=schema.dtype)
    for index, point in enumerate(points):
        data[index]["x"] = point[0]
        data[index]["y"] = point[1]
        data[index]["z"] = point[2]
        data[index]["intensity"] = point[3] if len(point) > 3 else index
    return PcdCloud(schema, data, "ascii")


def _recipe(path: Path, operations, *, output_data="binary", seed=7):
    path.write_text(
        yaml.safe_dump(
            {
                "schema": "agt_pointcloud_processing_recipe/v1",
                "recipe_id": "fixture_processing",
                "frame_id": "map",
                "output_data": output_data,
                "random_seed": seed,
                "operations": operations,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return path


def test_pcd_ascii_to_binary_roundtrip_preserves_schema_and_values(tmp_path):
    source = tmp_path / "source.pcd"
    write_pcd(
        source,
        _cloud([(1.0, 2.0, 3.0, 4.0), (-1.0, 0.5, 2.5, 9.0)]),
        data_mode="ascii",
    )
    loaded = read_pcd(source)
    assert loaded.data_mode == "ascii"
    assert loaded.schema.fields == ("x", "y", "z", "intensity")
    np.testing.assert_allclose(loaded.xyz(), [[1.0, 2.0, 3.0], [-1.0, 0.5, 2.5]])
    np.testing.assert_allclose(loaded.points["intensity"], [4.0, 9.0])

    binary = tmp_path / "binary.pcd"
    write_pcd(binary, loaded, data_mode="binary")
    binary_loaded = read_pcd(binary)
    assert binary_loaded.data_mode == "binary"
    np.testing.assert_allclose(binary_loaded.xyz(), loaded.xyz())
    np.testing.assert_allclose(binary_loaded.points["intensity"], loaded.points["intensity"])


def test_processing_is_immutable_and_deterministic_for_same_input_recipe(tmp_path):
    source = tmp_path / "source.pcd"
    write_pcd(
        source,
        _cloud(
            [
                (-2.0, 0.0, 0.1, 1.0),
                (0.01, 0.01, 0.10, 2.0),
                (0.02, 0.02, 0.11, 3.0),
                (0.35, 0.00, 0.12, 4.0),
                (0.36, 0.01, 0.13, 5.0),
                (1.5, 0.0, 0.2, 6.0),
            ]
        ),
        data_mode="binary",
    )
    original_hash = sha256_file(source)
    recipe = _recipe(
        tmp_path / "recipe.yaml",
        [
            {
                "type": "crop_box",
                "parameters": {"min": [-1.0, -1.0, 0.0], "max": [1.0, 1.0, 1.0]},
            },
            {"type": "voxel_downsample", "parameters": {"leaf_size_m": 0.2}},
        ],
    )

    first = process_pointcloud(source, recipe, tmp_path / "run_a")
    second = process_pointcloud(source, recipe, tmp_path / "run_b")

    assert sha256_file(source) == original_hash
    assert first.output_sha256 == second.output_sha256
    assert first.input_points == 6
    assert first.output_points == 2
    assert validate_pointcloud_processing(first.run_dir).valid
    assert validate_pointcloud_processing(second.run_dir).valid
    with pytest.raises(AssetContractError) as raised:
        process_pointcloud(source, recipe, first.run_dir)
    assert raised.value.code == "pointcloud_output_exists"


def test_delete_polygon_is_replayable_manual_edit_primitive(tmp_path):
    source = tmp_path / "source.pcd"
    write_pcd(
        source,
        _cloud(
            [
                (0.0, 0.0, 0.5, 1.0),
                (0.5, 0.5, 0.5, 2.0),
                (2.0, 2.0, 0.5, 3.0),
            ]
        ),
    )
    recipe = _recipe(
        tmp_path / "delete.yaml",
        [
            {
                "type": "delete_polygon",
                "parameters": {
                    "polygon_xy": [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
                    "z_min": 0.0,
                    "z_max": 1.0,
                },
            }
        ],
    )
    result = process_pointcloud(source, recipe, tmp_path / "delete_run")
    output = read_pcd(result.output_path)
    assert result.output_points == 1
    np.testing.assert_allclose(output.xyz()[0], [2.0, 2.0, 0.5])


def test_sor_removes_isolated_far_point(tmp_path):
    cluster = [
        (0.00, 0.00, 0.0, 1.0),
        (0.05, 0.00, 0.0, 2.0),
        (0.00, 0.05, 0.0, 3.0),
        (0.05, 0.05, 0.0, 4.0),
        (0.02, 0.02, 0.0, 5.0),
        (5.00, 5.00, 5.0, 6.0),
    ]
    source = tmp_path / "source.pcd"
    write_pcd(source, _cloud(cluster))
    recipe = _recipe(
        tmp_path / "sor.yaml",
        [{"type": "sor", "parameters": {"mean_k": 3, "stddev_mul": 1.0}}],
    )
    result = process_pointcloud(source, recipe, tmp_path / "sor_run")
    xyz = read_pcd(result.output_path).xyz()
    assert xyz.shape[0] < len(cluster)
    assert not np.any(np.all(np.isclose(xyz, [5.0, 5.0, 5.0]), axis=1))


def test_ground_separation_is_seeded_and_keeps_non_ground(tmp_path):
    points = []
    intensity = 0.0
    for x in (-1.0, -0.5, 0.0, 0.5, 1.0):
        for y in (-1.0, -0.5, 0.0, 0.5, 1.0):
            points.append((x, y, 0.0, intensity))
            intensity += 1.0
    points += [(-0.2, 0.0, 0.8, 100.0), (0.3, 0.2, 1.1, 101.0)]
    source = tmp_path / "ground.pcd"
    write_pcd(source, _cloud(points))
    recipe = _recipe(
        tmp_path / "ground.yaml",
        [
            {
                "type": "ground_separation",
                "parameters": {
                    "method": "plane_ransac",
                    "distance_threshold_m": 0.03,
                    "max_iterations": 80,
                    "max_tilt_deg": 10.0,
                    "keep": "nonground",
                },
            }
        ],
        seed=42,
    )
    first = process_pointcloud(source, recipe, tmp_path / "ground_a")
    second = process_pointcloud(source, recipe, tmp_path / "ground_b")
    assert first.output_sha256 == second.output_sha256
    output = read_pcd(first.output_path)
    assert output.points.shape[0] == 2
    assert np.all(output.xyz()[:, 2] > 0.5)


def test_validation_detects_output_tamper(tmp_path):
    source = tmp_path / "source.pcd"
    write_pcd(source, _cloud([(0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 2.0)]))
    recipe = _recipe(
        tmp_path / "recipe.yaml",
        [{"type": "height_range", "parameters": {"min_z": -1.0, "max_z": 1.0}}],
    )
    result = process_pointcloud(source, recipe, tmp_path / "run")
    assert validate_pointcloud_processing(result.run_dir).valid
    with result.output_path.open("ab") as stream:
        stream.write(b"tamper")
    compliance = validate_pointcloud_processing(result.run_dir)
    assert not compliance.valid
    assert "pointcloud_output_hash_mismatch" in compliance.errors
