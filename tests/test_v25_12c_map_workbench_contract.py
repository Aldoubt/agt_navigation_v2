from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/agt_map_workbench"
MODEL = PACKAGE / "agt_map_workbench/model.py"
FRAME = PACKAGE / "agt_map_workbench/frame_calibration.py"
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


def test_authoring_feedback_is_explicit_and_zoom_stable():
    app = _read(APP)
    view = _read(VIEW)
    for token in (
        "当前可见采样点",
        "当前多边形顶点顺序",
        "执行顺序",
        "黄色编号",
        "ItemIgnoresTransformations",
        "padded_bounding_rect",
    ):
        assert token in app or token in view
    assert "visible_sample_count" in view
    assert "sample_count" in view
    assert "Qt.CrossCursor" in view
    assert "ratio=0.10" in app
    assert "minimum_margin=1.0" in app


def test_display_quality_controls_are_preview_only():
    app = _read(APP)
    view = _read(VIEW)
    for token in (
        "深色背景",
        "浅色背景",
        "按高度着色",
        "按强度着色",
        "显示 15 万点",
        "显示 30 万点",
        "显示 60 万点",
        "仅影响预览，不修改正式 PCD",
    ):
        assert token in app
    assert "for size in (1, 2, 3, 4)" in app
    assert 'f"点大小 {size}px"' in app
    assert 'set_background_mode("dark")' in view
    assert '"height", "intensity", "mono"' in view
    assert "setWidthF(self._point_size_px)" in view


def test_map_frame_calibration_is_separate_from_georeference():
    app = _read(APP)
    frame = _read(FRAME)
    for token in (
        "agt_map_frame_calibration/v1",
        "fit_horizontal_axis_from_corridor",
        "fit_vertical_axis_from_cylinder",
        "nearest_xyz_in_window",
        "solve_map_frame",
        "output_right_handed",
        '"status": "UNBOUND"',
        "ENU/UTM georeference is a separate calibration artifact",
    ):
        assert token in frame
    for ui_token in (
        "坐标系标定",
        "选择地图原点",
        "选择 X 参考墙两端",
        "选择 Z 参考立柱中心",
        "翻转 X 方向",
        "翻转 Z 方向",
        "导出 map_frame.yaml",
        "输入 X/Z 夹角",
    ):
        assert ui_token in app
    assert "process_pointcloud" not in frame
    assert "rclpy" not in frame


def test_map_frame_is_orthogonalized_by_cross_products_not_raw_mouse_axes():
    frame = _read(FRAME)
    assert "x_projected" in frame
    assert "np.cross(z_axis, x_axis)" in frame
    assert "np.cross(y_axis, z_axis)" in frame
    assert "basis_source_from_map" in frame
    assert "rotation_map_from_source" in frame


def test_operator_ui_is_chinese_but_machine_contract_remains_stable():
    app = _read(APP)
    for token in (
        "AGT 地图工作台",
        "打开 PCD 点云",
        "开始绘制多边形",
        "完成多边形并加入处理流程",
        "导出处理 Recipe YAML",
        "执行完整分辨率不可变处理",
        "点云适配窗口",
    ):
        assert token in app
    for machine_token in ("delete_polygon", "crop_polygon", "process_pointcloud"):
        assert machine_token in app


def test_workbench_does_not_hardcode_user_workspace_path():
    app = _read(APP)
    assert "/home/yangxuan" not in app
    assert 'Path.cwd() / "runtime" / "maps"' in app


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
    assert 'file(REMOVE "${AGT_MAP_WORKBENCH_CLI_DIR}/agt_map_workbench.py")' in cmake
    assert LAUNCHER.name != "agt_map_workbench.py"
    assert "from agt_map_workbench.app import main" in launcher
    assert not (PACKAGE / "scripts/agt_map_workbench.py").exists()


def test_frame_calibration_tests_are_registered():
    cmake = _read(CMAKE)
    assert "test_frame_calibration" in cmake
    assert "test/test_frame_calibration.py" in cmake


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
