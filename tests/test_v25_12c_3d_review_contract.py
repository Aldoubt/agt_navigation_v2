from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKBENCH = ROOT / "src/agt_map_workbench"
OFFLINE = ROOT / "src/agt_offline_assets"
REVIEW_3D = WORKBENCH / "agt_map_workbench/review_3d.py"
REVIEW_APP = WORKBENCH / "agt_map_workbench/review_workbench.py"
VEHICLE = OFFLINE / "agt_offline_assets/vehicle_corridor.py"
OFFLINE_CMAKE = OFFLINE / "CMakeLists.txt"
LAUNCHER = WORKBENCH / "scripts/map_workbench_launcher.py"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_3d_review_is_separate_from_2d_authoring_authority():
    review = _read(REVIEW_3D)
    app = _read(REVIEW_APP)
    assert "2D canvas remains the authoring authority" in review
    assert "ReviewMapWorkbenchWindow" in app
    assert "2D 编辑 / 分析" in app
    assert "3D 审查" in app
    assert "AgriculturalMapWorkbenchWindow" in app
    assert "process_pointcloud" not in review
    assert "apply_navigation_overrides" not in review
    assert "rclpy" not in review


def test_3d_review_uses_deterministic_sample_and_mouse_camera_controls():
    review = _read(REVIEW_3D)
    for token in (
        "np.linspace",
        "sample_limit",
        "3D 6 万点",
        "3D 15 万点",
        "3D 30 万点",
        "左键旋转",
        "右键平移",
        "滚轮缩放",
        "顶视",
        "前视",
        "侧视",
        "等轴测",
    ):
        assert token in review
    for forbidden in ("open3d", "vtk", "pyvista", "rclpy"):
        assert f"import {forbidden}" not in review.lower()


def test_3d_navigation_layers_use_ground_height_not_flat_z_zero():
    review = _read(REVIEW_3D)
    for token in (
        "navigation.ground_height_m",
        "row_centerline",
        "row_structural_band",
        "refined_aisle",
        "aisle_centerline",
        "vehicle_corridor",
        "z_offset_m",
    ):
        assert token in review
    assert '"ground", self._grid_xyz' in review


def test_vehicle_corridor_is_offline_review_contract_and_keeps_conflicts():
    vehicle = _read(VEHICLE)
    app = _read(REVIEW_APP)
    for token in (
        "VehicleCorridorConfig",
        "VehicleCorridorResult",
        "required_envelope_mask",
        "corridor_mask",
        "conflict_mask",
        "vehicle_width_m",
        "lateral_safety_margin_m",
        "derive_vehicle_corridor",
    ):
        assert token in vehicle
    assert "derive_vehicle_corridor" in app
    assert "result.occupancy =" not in vehicle
    assert "final OccupancyGrid" in vehicle


def test_vehicle_corridor_test_is_registered_and_launcher_enters_review_composition():
    cmake = _read(OFFLINE_CMAKE)
    launcher = _read(LAUNCHER)
    assert "test_vehicle_corridor" in cmake
    assert "test/test_vehicle_corridor.py" in cmake
    assert "from agt_map_workbench.review_workbench import main" in launcher
