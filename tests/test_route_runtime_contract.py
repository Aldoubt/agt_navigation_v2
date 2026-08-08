from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NAVIGATION = ROOT / "src/agt_navigation"
BRINGUP = ROOT / "src/agt_bringup"


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
    adapter = _read(NAVIGATION / "agt_navigation" / "nav2_follow_path_adapter.py")
    backend = _read(NAVIGATION / "agt_navigation" / "route_backend.py")

    assert not (action_dir / "ExecuteRouteTask.action").exists()
    assert not (action_dir / "ExecuteNavigationTask.action").exists()
    for text in (core, adapter, backend):
        assert "ComputePathToPose" not in text
        assert "nav2_planner" not in text
        assert "cmd_vel" not in text
        assert "TransformBroadcaster" not in text

    assert "FollowPath" in adapter
    assert 'frame_id = "odom"' in adapter
    assert "controller_id_forward" in adapter
    assert "controller_id_reverse" in adapter


def test_route_backend_uses_rclpy_future_not_asyncio_loop():
    backend = _read(NAVIGATION / "agt_navigation" / "route_backend.py")

    assert "from rclpy.task import Future" in backend
    assert "await completion_future" in backend
    assert "import asyncio" not in backend
    assert "asyncio.sleep" not in backend


def test_task_route_binding_is_optional_exact_and_fail_closed():
    resolver = _read(NAVIGATION / "agt_navigation" / "route_task_binding.py")
    contract = _read(ROOT / "docs/interfaces/task_route_execution_binding.md")

    assert 'f"{task.task_group_id}.route.yaml"' in resolver
    assert "task_content_sha256" in resolver
    assert "route_manifest_sha256" in resolver
    assert "expected_vehicle_profile_sha256" in resolver
    assert "map_content_sha256" in resolver
    assert "return None" in resolver

    assert "no  -> MAP backend" in contract
    assert "yes -> validate binding fail-closed" in contract
    assert "MUST NOT silently fall back to MAP" in contract
    assert "TaskGroup schema field" in contract


def test_public_navigation_capability_selects_map_or_route_without_schema_change():
    capability = _read(NAVIGATION / "scripts" / "navigation_capability_server.py")
    nav_launch = _read(NAVIGATION / "launch" / "navigation.launch.py")
    system_launch = _read(BRINGUP / "launch" / "navigation_system.launch.py")
    action = _read(ROOT / "src/agt_interfaces/action/ExecuteWaypointTask.action")

    assert "class NavigationCapabilityServer" in capability
    assert "await super()._execute(goal_handle)" in capability
    assert "RouteBackendExecutor" in capability
    assert "FollowPath" in capability
    assert "execution_vehicle_profile" in capability
    assert "route_acceptance" in capability

    assert 'executable="navigation_capability_server.py"' in nav_launch
    assert '"execution_vehicle_profile"' in nav_launch
    assert '"execution_vehicle_profile": LaunchConfiguration("platform_profile")' in system_launch

    # Public Action stays the frozen formal TaskGroup contract; no runtime mode or
    # Route path is pushed into the ROS interface.
    assert "task_group_id" in action
    assert "expected_content_sha256" in action
    assert "route_id" not in action
    assert "navigation_mode" not in action


def test_runtime_gate_action_tests_are_registered():
    cmake = _read(NAVIGATION / "CMakeLists.txt")
    runtime_gate_test = NAVIGATION / "test" / "test_navigation_capability_runtime_gates.py"

    assert "test_navigation_capability_runtime_gates" in cmake
    assert runtime_gate_test.is_file()
    content = _read(runtime_gate_test)
    for token in (
        "test_parent_cancel_reaches_active_route_follow_path_child",
        "SAFETY_NOT_READY",
        "LOCALIZATION_NOT_READY",
        "TASK_READINESS_NOT_READY",
        "child_canceled",
    ):
        assert token in content


def test_controller_only_smoke_uses_real_follow_path_without_planner_server():
    launch = _read(NAVIGATION / "launch" / "route_controller_smoke.launch.py")
    client = _read(NAVIGATION / "scripts" / "route_controller_smoke.py")
    cmake = _read(NAVIGATION / "CMakeLists.txt")

    assert 'package="nav2_controller"' in launch
    assert 'executable="controller_server"' in launch
    assert 'package="nav2_collision_monitor"' in launch
    assert 'executable="collision_monitor"' in launch
    assert 'executable="differential_drive_simulator.py"' in launch
    assert 'executable="tracked_safety_controller.py"' in launch
    assert "planner_server" not in launch
    assert "bt_navigator" not in launch
    assert "map_server" not in launch

    assert "FollowPath" in client
    assert 'path.header.frame_id = "odom"' in client
    assert '"global_planner_requests": 0' in client
    assert "route_controller_smoke.py" in cmake

    # The smoke client is a newly generated script. GitHub contents writes may
    # create it as 0644, so symlink-install must materialize an executable copy.
    assert "AGT_NAVIGATION_GENERATED_SCRIPT_DIR" in cmake
    assert "FILE_PERMISSIONS" in cmake
    assert "route_controller_smoke.py" in cmake
    assert "OWNER_EXECUTE" in cmake
    assert "GROUP_EXECUTE" in cmake
    assert "WORLD_EXECUTE" in cmake


def test_full_route_system_smoke_uses_formal_action_assets_and_real_controller_only():
    launch = _read(NAVIGATION / "launch" / "route_system_smoke.launch.py")
    fixture = _read(NAVIGATION / "scripts" / "route_system_smoke_fixture.py")
    client = _read(NAVIGATION / "scripts" / "route_system_smoke.py")
    profile = _read(NAVIGATION / "config" / "route_smoke_vehicle.yaml")
    cmake = _read(NAVIGATION / "CMakeLists.txt")

    assert "route_controller_smoke.launch.py" in launch
    assert 'executable="route_system_smoke_fixture.py"' in launch
    assert 'executable="navigation_capability_server.py"' in launch
    assert '"require_map": True' in launch
    assert '"require_safety_ready": True' in launch
    assert '"require_localization_valid": False' in launch
    assert '"require_task_readiness": False' in launch
    assert "planner_server" not in launch
    assert "bt_navigator" not in launch
    assert "map_server" not in launch

    for token in (
        "TaskGroup(",
        '"state": "READY"',
        '"status": "READY"',
        "route_manifest_sha256",
        "platform_profile_sha256",
        "/agt/map/global_occupancy",
        "/agt/maps/active",
    ):
        assert token in fixture

    assert "ExecuteWaypointTask" in client
    assert "expected_content_sha256" in client
    assert "global_planner_requests" in client
    assert "measured_displacement_m" in client

    assert "SOFTWARE_ONLY" in profile
    assert "route_acceptance:" in profile
    assert "enabled: true" in profile
    assert "verified: false" in profile

    for script in (
        "route_controller_smoke.py",
        "route_system_smoke.py",
        "route_system_smoke_fixture.py",
    ):
        assert script in cmake


def test_route_runtime_package_tests_are_registered():
    cmake = _read(NAVIGATION / "CMakeLists.txt")
    for target in (
        "test_route_runtime",
        "test_nav2_follow_path_adapter",
        "test_route_task_binding",
        "test_navigation_capability_server",
        "test_navigation_capability_runtime_gates",
    ):
        assert target in cmake

    for relative in (
        "test/test_route_runtime.py",
        "test/test_nav2_follow_path_adapter.py",
        "test/test_route_task_binding.py",
        "test/test_navigation_capability_server.py",
        "test/test_navigation_capability_runtime_gates.py",
    ):
        assert (NAVIGATION / relative).is_file()
