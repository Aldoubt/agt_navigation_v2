from __future__ import annotations

from pathlib import Path


LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "benchmark_run.launch.py"
SCRIPT_DIR = LAUNCH_PATH.parents[1] / "scripts"


def test_benchmark_launch_exposes_and_forwards_semantic_map():
    text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("semantic_map", default_value="")' in text
    assert 'LaunchConfiguration("semantic_map").perform(context).strip()' in text
    assert 'run_args.extend(["--semantic-map"' in text


def test_benchmark_runner_waits_for_lifecycle_managed_planner_startup():
    text = LAUNCH_PATH.read_text(encoding="utf-8")
    assert "TimerAction(period=5.0, actions=[runner])" in text


def test_ros2_run_scripts_are_executable():
    """CMake install(PROGRAMS) must preserve executable entry points."""
    scripts = sorted(SCRIPT_DIR.glob("*.py"))
    assert scripts
    assert all(path.stat().st_mode & 0o111 for path in scripts)
