import hashlib
import importlib.util
from pathlib import Path

from agt_interfaces.srv import ListMapVersions, ManageMapVersion
import rclpy
import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "map_manager_node.py"
SPEC = importlib.util.spec_from_file_location("map_manager_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_generated_map_services_return_typed_empty_state(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"runtime_dir:={tmp_path}"])
    node = MODULE.MapManagerNode()
    try:
        listed = node._list(ListMapVersions.Request(), ListMapVersions.Response())
        assert listed.success
        assert listed.error_code == ListMapVersions.Response.ERROR_NONE
        assert listed.versions == []
        active = ManageMapVersion.Request()
        active.operation = ManageMapVersion.Request.OP_GET_ACTIVE
        response = node._manage(active, ManageMapVersion.Response())
        assert response.success
        assert not response.version.active
        assert response.version.map_version_id == ""
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_destructive_map_operation_requires_confirmation(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"runtime_dir:={tmp_path}"])
    node = MODULE.MapManagerNode()
    try:
        request = ManageMapVersion.Request()
        request.operation = ManageMapVersion.Request.OP_PURGE
        request.map_version_id = "missing"
        response = node._manage(request, ManageMapVersion.Response())
        assert not response.success
        assert response.error_code == ManageMapVersion.Response.ERROR_NOT_FOUND
        request = ManageMapVersion.Request()
        request.operation = ManageMapVersion.Request.OP_IMPORT_CANDIDATE
        response = node._manage(request, ManageMapVersion.Response())
        assert not response.success
        assert response.error_code == ManageMapVersion.Response.ERROR_INVALID_REQUEST
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mapping_candidate_import_returns_manager_owned_asset_paths(tmp_path):
    source = tmp_path / "candidate"
    source.mkdir()
    image = source / "candidate.pgm"
    image.write_bytes(b"P5\n2 2\n255\n" + bytes([254, 205, 0, 254]))
    map_yaml = source / "candidate.yaml"
    map_yaml.write_text(
        yaml.safe_dump({
            "image": image.name,
            "mode": "trinary",
            "resolution": 0.05,
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }),
        encoding="utf-8",
    )
    pcd = source / "localization_map.pcd"
    pcd.write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
        encoding="ascii",
    )
    pcd_hash = "sha256:" + hashlib.sha256(pcd.read_bytes()).hexdigest()
    record = source / "localization_map.processing.yaml"
    record.write_text(
        yaml.safe_dump({
            "state": "ready",
            "map_file": pcd.name,
            "pcd_sha256": pcd_hash,
        }),
        encoding="utf-8",
    )

    runtime = tmp_path / "runtime"
    rclpy.init(args=["--ros-args", "-p", f"runtime_dir:={runtime}"])
    node = MODULE.MapManagerNode()
    try:
        request = ManageMapVersion.Request()
        request.operation = ManageMapVersion.Request.OP_IMPORT_CANDIDATE
        request.map_id = "greenhouse_01"
        request.candidate_map_yaml = str(map_yaml)
        request.localization_pcd = str(pcd)
        request.processing_record = str(record)
        response = node._manage(request, ManageMapVersion.Response())
        assert response.success
        assert response.version.valid
        assert response.version.state == response.version.STATE_READY
        assert response.version.map_hash == pcd_hash
        assert Path(response.version.navigation_yaml).is_file()
        assert Path(response.version.tasks_directory).is_dir()
        assert source not in Path(response.version.navigation_yaml).parents
    finally:
        node.destroy_node()
        rclpy.shutdown()
