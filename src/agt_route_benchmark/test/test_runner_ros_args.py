from __future__ import annotations

import importlib.util
from pathlib import Path


RUNNER_PATH = Path(__file__).resolve().parents[1] / "scripts" / "route_benchmark_run.py"


def _load_runner_module():
    spec = importlib.util.spec_from_file_location("route_benchmark_run_under_test", RUNNER_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_benchmark_cli_args_removes_launch_ros_arguments():
    runner = _load_runner_module()
    argv = [
        "route_benchmark_run.py",
        "--site",
        "greenhouse_01",
        "--scenario",
        "scenario.yaml",
        "--planner",
        "astar",
        "--ros-args",
        "-r",
        "__node:=agt_route_benchmark_runner",
    ]

    assert runner._benchmark_cli_args(argv) == [
        "--site",
        "greenhouse_01",
        "--scenario",
        "scenario.yaml",
        "--planner",
        "astar",
    ]


def test_benchmark_cli_args_keeps_unknown_benchmark_argument_for_argparse():
    runner = _load_runner_module()
    argv = [
        "route_benchmark_run.py",
        "--platfrom-profile",
        "bad.yaml",
        "--ros-args",
        "-r",
        "__node:=agt_route_benchmark_runner",
    ]

    assert runner._benchmark_cli_args(argv) == [
        "--platfrom-profile",
        "bad.yaml",
    ]
