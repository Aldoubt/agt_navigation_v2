from pathlib import Path

import pytest
import yaml

from agt_map_manager.facade import (
    MapBusinessFacade, experiment_map_references, resolve_assets, state_name,
)


class FakeRegistry:
    def __init__(self, rows):
        self.rows = {row["version_id"]: dict(row) for row in rows}
        self.calls = []

    def list_versions(self, *, map_id=None, state=None, include_deleted=False):
        result = list(self.rows.values())
        if map_id:
            result = [row for row in result if row["map_id"] == map_id]
        if state:
            result = [row for row in result if row["state"] == state]
        if not include_deleted:
            result = [row for row in result if not row.get("deleted")]
        return result

    def _row(self, version_id):
        if version_id not in self.rows:
            raise KeyError(version_id)
        return dict(self.rows[version_id])

    def soft_delete(self, version_id):
        self.calls.append(("soft_delete", version_id))
        self.rows[version_id]["deleted"] = 1
        self.rows[version_id]["state"] = "ARCHIVED"

    def purge(self, version_id):
        self.calls.append(("purge", version_id))
        del self.rows[version_id]


def row(version_id="map_20260728_120000_1234abcd", **values):
    return {
        "map_id": "map_a",
        "version_id": version_id,
        "state": "ARCHIVED",
        "active": 0,
        "pinned": 0,
        "deleted": 0,
        **values,
    }


def test_state_filter_rejects_unknown_and_filters_deleted(tmp_path):
    registry = FakeRegistry([row(), row("map_20260728_120001_1234abce", deleted=1)])
    facade = MapBusinessFacade(registry, tmp_path)
    assert state_name(3) == "READY"
    with pytest.raises(ValueError):
        state_name(99)
    assert [item["version_id"] for item in facade.list_rows(state=6)] == [
        "map_20260728_120001_1234abce"
    ]


def test_experiment_reference_and_confirmation_block_deletion(tmp_path):
    version_id = "map_20260728_120000_1234abcd"
    experiment = tmp_path / "exp_01"
    experiment.mkdir()
    (experiment / "manifest.yaml").write_text(
        yaml.safe_dump({"active_map": {"map_version_id": version_id}}),
        encoding="utf-8",
    )
    registry = FakeRegistry([row(version_id)])
    facade = MapBusinessFacade(registry, tmp_path)
    assert experiment_map_references(tmp_path) == {version_id}
    with pytest.raises(PermissionError):
        facade.manage(6, version_id, False)
    with pytest.raises(ValueError, match="experiment"):
        facade.manage(6, version_id, True)
    assert registry.calls == []


def test_unreadable_experiment_reference_fails_closed(tmp_path):
    version_id = "map_20260728_120000_1234abcd"
    experiment = tmp_path / "exp_01"
    experiment.mkdir()
    (experiment / "manifest.yaml").write_text("active_map: [\n", encoding="utf-8")
    facade = MapBusinessFacade(FakeRegistry([row(version_id)]), tmp_path)
    with pytest.raises(ValueError, match="cannot verify"):
        facade.manage(6, version_id, True)


def test_purge_requires_soft_deleted_version_and_confirmation(tmp_path):
    version_id = "map_20260728_120000_1234abcd"
    registry = FakeRegistry([row(version_id, deleted=1)])
    facade = MapBusinessFacade(registry, tmp_path)
    with pytest.raises(PermissionError):
        facade.manage(7, version_id, False)
    assert facade.manage(7, version_id, True) is None
    assert registry.calls == [("purge", version_id)]


def test_resolve_assets_returns_manager_owned_absolute_paths(tmp_path):
    root = tmp_path / "map_a" / "versions" / "map_20260728_120000_1234abcd"
    (root / "navigation").mkdir(parents=True)
    (root / "pointcloud").mkdir()
    (root / "tasks").mkdir()
    for relative in (
        "navigation/map.yaml",
        "pointcloud/localization_map.pcd",
        "pointcloud/localization_map.processing.yaml",
    ):
        (root / relative).write_text("asset", encoding="utf-8")
    manifest = root / "manifest.yaml"
    manifest.write_text(
        yaml.safe_dump({
            "assets": {
                "navigation_yaml": {"path": "navigation/map.yaml"},
                "localization_pcd": {"path": "pointcloud/localization_map.pcd"},
                "processing_record": {"path": "pointcloud/localization_map.processing.yaml"},
            }
        }),
        encoding="utf-8",
    )
    assets = resolve_assets({"manifest_path": str(manifest)})
    assert Path(assets["navigation_yaml"]).is_absolute()
    assert assets["tasks_directory"] == str((root / "tasks").resolve())
