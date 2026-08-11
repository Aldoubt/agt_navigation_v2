from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/agt_map_workbench"
MODEL = PACKAGE / "agt_map_workbench/model.py"
APP = PACKAGE / "agt_map_workbench/app.py"
VIEW = PACKAGE / "agt_map_workbench/view.py"
PACKAGE_XML = PACKAGE / "package.xml"
CMAKE = PACKAGE / "CMakeLists.txt"
LAUNCHER = PACKAGE / "scripts/map_workbench_launcher.py"
README = PACKAGE / "README.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_workbench_is_pure_offline_package():
    package_xml = _read(PACKAGE_XML)
    app = _read(APP)
    assert "agt_offline_assets" in package_xml
    assert "python3-pyqt5" in package_xml
    assert "rclpy" not in package_xml
    assert "rclpy" not in app
    assert "cmd_vel" not in app


def test_visual_polygon_serializes_existing_processing_contract():
    model = _read(MODEL)
    for token in (
        "agt_pointcloud_processing_recipe/v1",
        "delete_polygon",
        "crop_polygon",
        "polygon_xy",
        "z_min",
        "z_max",
    ):
        assert token in model


def test_display_sampling_is_separate_from_full_resolution_processing():
    view = _read(VIEW)
    app = _read(APP)
    readme = _read(README)
    assert "np.linspace" in view
    assert "sample_limit" in view
    assert "process_pointcloud(self.input_path" in app
    assert "Display sampling never changes the formal processing input" in readme


def test_ros2_run_launcher_is_executable_and_does_not_shadow_python_package():
    cmake = _read(CMAKE)
    launcher = _read(LAUNCHER)
    assert "AGT_MAP_WORKBENCH_CLI_DIR" in cmake
    assert "FILE_PERMISSIONS" in cmake
    assert "OWNER_EXECUTE" in cmake
    assert "GROUP_EXECUTE" in cmake
    assert "WORLD_EXECUTE" in cmake
    assert "map_workbench_launcher.py" in cmake
    assert "RENAME agt_map_workbench" in cmake
    assert LAUNCHER.name != "agt_map_workbench.py"
    assert "from agt_map_workbench.app import main" in launcher
    assert not (PACKAGE / "scripts/agt_map_workbench.py").exists()


def test_workbench_does_not_add_runtime_ros_interfaces():
    action_dir = ROOT / "src/agt_interfaces/action"
    service_dir = ROOT / "src/agt_interfaces/srv"
    for name in (
        "EditMap.action",
        "EditPointCloud.action",
        "MapWorkbench.action",
        "EditMap.srv",
        "EditPointCloud.srv",
        "MapWorkbench.srv",
    ):
        assert not (action_dir / name).exists()
        assert not (service_dir / name).exists()
