from pathlib import Path

import pytest
import yaml

from agt_map_workbench.resource_bundle import (
    ResourceBundleGroup,
    ResourceBundleSelection,
    export_map_resource_bundle,
)


def _writer(relative_path: str, payload: bytes):
    def write(root: Path) -> None:
        target = root / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)

    return write


def test_recommended_selection_does_not_copy_original_pcd():
    selection = ResourceBundleSelection.recommended(
        available_groups={
            "processed_pointcloud",
            "navigation_revision",
            "terrain_layers",
            "structure_layers",
            "traversability_12f",
            "recipe",
            "map_frame",
            "site_boundary",
        }
    )
    assert selection.copy_original_pcd is False
    assert "processed_pointcloud" in selection.selected_groups
    assert "navigation_revision" in selection.selected_groups
    assert "terrain_layers" in selection.selected_groups
    assert "structure_layers" in selection.selected_groups
    assert "recipe" in selection.selected_groups
    assert "map_frame" in selection.selected_groups
    assert "site_boundary" in selection.selected_groups
    assert "traversability_12f" not in selection.selected_groups


def test_export_writes_only_selected_groups_and_references_original_without_copy(tmp_path: Path):
    original = tmp_path / "raw_greenhouse.pcd"
    original.write_bytes(b"raw-pcd")
    current = tmp_path / "processed.pcd"
    current.write_bytes(b"processed-pcd")

    groups = [
        ResourceBundleGroup(
            key="processed_pointcloud",
            label="processed",
            available=True,
            default_selected=True,
            writer=_writer("pointcloud/processed.pcd", b"processed-pcd"),
        ),
        ResourceBundleGroup(
            key="terrain_layers",
            label="terrain",
            available=True,
            default_selected=True,
            writer=_writer("layers/terrain/ground_height.npy", b"terrain"),
        ),
        ResourceBundleGroup(
            key="traversability_12f",
            label="12f",
            available=True,
            default_selected=False,
            writer=_writer("layers/traversability/navigation_map_12f.pgm", b"12f"),
        ),
    ]
    selection = ResourceBundleSelection(
        selected_groups=frozenset({"processed_pointcloud", "terrain_layers"}),
        copy_original_pcd=False,
    )

    output = export_map_resource_bundle(
        tmp_path / "exports",
        "greenhouse_01_assets_v01",
        groups=groups,
        selection=selection,
        current_pcd=current,
        original_pcd=original,
    )

    assert (output / "pointcloud" / "processed.pcd").read_bytes() == b"processed-pcd"
    assert (output / "layers" / "terrain" / "ground_height.npy").is_file()
    assert not (output / "layers" / "traversability").exists()
    assert not (output / "pointcloud" / "original.pcd").exists()

    manifest = yaml.safe_load((output / "map_resource_manifest.yaml").read_text())
    assert manifest["schema"] == "agt_map_resource_bundle/v1"
    assert manifest["selection"]["copy_original_pcd"] is False
    assert manifest["source"]["original_pcd"]["path"] == str(original.resolve())
    assert len(manifest["source"]["original_pcd"]["sha256"]) == 64
    paths = {asset["path"] for asset in manifest["assets"]}
    assert "pointcloud/processed.pcd" in paths
    assert "layers/terrain/ground_height.npy" in paths
    assert all("traversability" not in path for path in paths)


def test_copy_original_pcd_only_when_explicitly_enabled(tmp_path: Path):
    original = tmp_path / "raw.pcd"
    original.write_bytes(b"raw")
    selection = ResourceBundleSelection(
        selected_groups=frozenset(),
        copy_original_pcd=True,
    )
    output = export_map_resource_bundle(
        tmp_path / "exports",
        "bundle",
        groups=[],
        selection=selection,
        original_pcd=original,
    )
    copied = output / "pointcloud" / "original.pcd"
    assert copied.read_bytes() == b"raw"
    manifest = yaml.safe_load((output / "map_resource_manifest.yaml").read_text())
    assert manifest["source"]["original_pcd"]["copied"] is True
    assert any(asset["path"] == "pointcloud/original.pcd" for asset in manifest["assets"])


def test_export_is_immutable_and_writer_failure_leaves_no_final_bundle(tmp_path: Path):
    parent = tmp_path / "exports"
    parent.mkdir()
    existing = parent / "existing"
    existing.mkdir()
    with pytest.raises(FileExistsError):
        export_map_resource_bundle(
            parent,
            "existing",
            groups=[],
            selection=ResourceBundleSelection(frozenset(), False),
        )

    def fail(_root: Path) -> None:
        raise RuntimeError("boom")

    with pytest.raises(RuntimeError, match="boom"):
        export_map_resource_bundle(
            parent,
            "failed",
            groups=[
                ResourceBundleGroup(
                    key="bad",
                    label="bad",
                    available=True,
                    default_selected=True,
                    writer=fail,
                )
            ],
            selection=ResourceBundleSelection(frozenset({"bad"}), False),
        )
    assert not (parent / "failed").exists()
    assert not any(path.name.startswith(".failed.tmp-") for path in parent.iterdir())
