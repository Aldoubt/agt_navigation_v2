from __future__ import annotations

from pathlib import Path


LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "benchmark_run.launch.py"


def test_benchmark_launch_exposes_and_forwards_semantic_map():
    text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("semantic_map", default_value="")' in text
    assert 'LaunchConfiguration("semantic_map").perform(context).strip()' in text
    assert 'run_args.extend(["--semantic-map"' in text
