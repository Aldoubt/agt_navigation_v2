from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def test_rescan_launch_is_fail_closed_and_uses_bootstrap_navigation_once():
    source = read("src/agt_bringup/launch/teach_mapping_rescan.launch.py")
    assert 'DeclareLaunchArgument("start_chassis", default_value="false")' in source
    assert 'DeclareLaunchArgument("execution_enabled", default_value="false")' in source
    assert 'DeclareLaunchArgument("record_bag", default_value="true")' in source
    assert 'DeclareLaunchArgument("start_gui", default_value="true")' in source
    assert 'DeclareLaunchArgument("rescan_max_speed_mps", default_value="0.10")' in source
    assert '"mode": "navigation"' in source
    assert '"map": bootstrap["map_yaml"]' in source
    assert '"global_map_pcd": bootstrap["localization_pcd"]' in source
    assert '"bag_profile": "teach_repeat"' in source
    assert '"auto_relocalize_on_start": "false"' in source
    assert '"auto_start": "false"' in source
    assert '"repeat_test.launch.py"' in source
    assert '"mapping_mode.launch.py"' not in source
    assert "motion_enable" not in source
    assert "/agt/teach/start" not in source


def test_rescan_speed_is_bounded_and_forwarded_to_executor():
    rescan = read("src/agt_bringup/launch/teach_mapping_rescan.launch.py")
    repeat = read("src/agt_teach_repeat/launch/repeat_test.launch.py")
    assert "0.02 <= speed <= 0.20" in rescan
    assert '"maximum_linear_speed_mps": str(speed)' in rescan
    assert 'LaunchConfiguration("maximum_linear_speed_mps")' in repeat


def test_system_manager_exposes_only_allowlisted_rescan_arguments():
    profiles = read("src/agt_system_manager/config/mode_profiles.yaml")
    assert "teach_rescan:" in profiles
    assert "mode: TEACH_RESCAN" in profiles
    assert "teach_mapping_rescan.launch.py" in profiles
    assert "build-candidate" not in profiles
