from pathlib import Path

import pytest
import yaml

from agt_map_workbench import WorkbenchRecipeModel


def test_polygon_delete_serializes_to_v25_12b_recipe(tmp_path):
    model = WorkbenchRecipeModel(recipe_id="greenhouse_edit", frame_id="map")
    model.add_polygon_volume(
        [(1.0, 2.0), (4.0, 2.0), (4.0, 5.0), (1.0, 5.0)],
        z_min=-1.5,
        z_max=4.5,
        delete=True,
    )
    output = model.write_yaml(tmp_path / "recipe.yaml")
    data = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert data["schema"] == "agt_pointcloud_processing_recipe/v1"
    assert data["recipe_id"] == "greenhouse_edit"
    assert data["frame_id"] == "map"
    assert data["operations"] == [
        {
            "type": "delete_polygon",
            "parameters": {
                "polygon_xy": [[1.0, 2.0], [4.0, 2.0], [4.0, 5.0], [1.0, 5.0]],
                "z_min": -1.5,
                "z_max": 4.5,
            },
        }
    ]


def test_polygon_crop_serializes_keep_inside():
    model = WorkbenchRecipeModel()
    model.add_polygon_volume(
        [(0, 0), (2, 0), (1, 1)], z_min=0.0, z_max=3.0, delete=False
    )
    operation = model.operations[0]
    assert operation.type == "crop_polygon"
    assert operation.parameters["keep_inside"] is True


def test_model_rejects_invalid_polygon_and_empty_recipe(tmp_path):
    model = WorkbenchRecipeModel()
    with pytest.raises(ValueError, match="at least three"):
        model.add_polygon_volume([(0, 0), (1, 0)], z_min=0, z_max=1, delete=True)
    with pytest.raises(ValueError, match="z_max"):
        model.add_polygon_volume([(0, 0), (1, 0), (0, 1)], z_min=2, z_max=1, delete=True)
    with pytest.raises(ValueError, match="at least one operation"):
        model.write_yaml(tmp_path / "empty.yaml")


def test_undo_and_clear_do_not_mutate_previous_operation_objects():
    model = WorkbenchRecipeModel()
    model.add_operation("remove_nonfinite")
    model.add_operation("voxel_downsample", {"leaf_size_m": 0.1})
    snapshot = model.operations
    removed = model.undo()
    assert removed.type == "voxel_downsample"
    assert len(model.operations) == 1
    assert len(snapshot) == 2
    model.clear()
    assert model.operations == ()
