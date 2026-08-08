from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NAVIGATION = ROOT / "src/agt_navigation"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_v25_09b_route_runtime_core_is_internal_and_odom_segment_based():
    core = _read(NAVIGATION / "agt_navigation" / "route_runtime.py")
    roadmap = _read(ROOT / "docs/roadmap/v25_09b_route_navigation_core.md")

    for token in (
        "RouteAsset",
        "RouteSegment",
        "MapOdomSnapshot",
        "RuntimePath",
        "VehicleTrackerAdapter",
        "global_planner_requests",
    ):
        assert token in core

    assert 'frame_id="odom"' in core
    assert "update_global_alignment" in core
    assert "next segment" in core
    assert "Global Planner" in roadmap
    assert "ExecuteWaypointTask" in roadmap


def test_route_runtime_does_not_add_public_action_or_native_global_planner_dependency():
    action_dir = ROOT / "src/agt_interfaces/action"
    core = _read(NAVIGATION / "agt_navigation" / "route_runtime.py")

    assert not (action_dir / "ExecuteRouteTask.action").exists()
    assert not (action_dir / "ExecuteNavigationTask.action").exists()
    assert "ComputePathToPose" not in core
    assert "nav2_planner" not in core
    assert "cmd_vel" not in core
    assert "TransformBroadcaster" not in core


def test_route_runtime_package_test_is_registered():
    cmake = _read(NAVIGATION / "CMakeLists.txt")
    assert "test_route_runtime" in cmake
    assert (NAVIGATION / "test" / "test_route_runtime.py").is_file()
