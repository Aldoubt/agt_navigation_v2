#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from agt_route_benchmark.adapters.fields2cover import Fields2CoverAdapter
from agt_route_benchmark.adapters.nav2_p2p import Nav2P2PAdapter
from agt_route_benchmark.adapters.proposed import ProposedAdapter
from agt_route_benchmark.contracts import ExperimentSpec, PathPoint
from agt_route_benchmark.experiment import ExperimentRunner
from agt_route_benchmark.graph_io import load_agricultural_graph
from agt_route_benchmark.path_io import read_path_csv
from agt_route_benchmark.profile import load_platform_profile
from agt_route_benchmark.renderer import render_route
from agt_route_benchmark.scenario import load_scenario


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _nav2_call():
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav2_simple_commander.robot_navigator import BasicNavigator
    except ImportError as exc:
        raise RuntimeError("Nav2 live mode requires ROS2 Humble + nav2_simple_commander") from exc

    rclpy.init()
    navigator = BasicNavigator()

    def pose(value):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = navigator.get_clock().now().to_msg()
        msg.pose.position.x = value[0]
        msg.pose.position.y = value[1]
        msg.pose.orientation.z = math.sin(value[2] / 2.0)
        msg.pose.orientation.w = math.cos(value[2] / 2.0)
        return msg

    def call(plugin_id, start, goal):
        path = navigator.getPath(pose(start), pose(goal), planner_id=plugin_id, use_start=True)
        if path is None:
            return ()
        return tuple(
            PathPoint(
                p.pose.position.x,
                p.pose.position.y,
                _quat_to_yaw(p.pose.orientation),
                "UNKNOWN",
                "P2P",
                "",
            )
            for p in path.poses
        )

    return call, navigator, rclpy


def _coverage_components(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, list):
        raise ValueError("coverage components JSON must be a list")
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one Paper I route benchmark cell")
    parser.add_argument("--site", default="greenhouse_01")
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--planner", required=True)
    parser.add_argument("--platform-profile", type=Path, default=Path("profiles/platforms/mk_mini.yaml"))
    parser.add_argument("--result-root", type=Path, default=Path("runtime/results/paper1_route_benchmark"))
    parser.add_argument("--formal", action="store_true")
    parser.add_argument("--run-id", default="run_001")
    parser.add_argument("--nav2-live", action="store_true")
    parser.add_argument("--graph", type=Path)
    parser.add_argument("--coverage-components-json", type=Path)
    args = parser.parse_args()

    scenario = load_scenario(args.scenario, formal=args.formal)
    profile = load_platform_profile(args.platform_profile)
    if not profile.preview_planning_enabled:
        raise SystemExit("platform profile disables preview planning")

    spec = ExperimentSpec(
        site_id=args.site,
        planner_id=args.planner,
        scenario=scenario,
        formal=args.formal,
        platform_profile=str(args.platform_profile),
        metadata={
            "platform_name": profile.name,
            "min_turning_radius_m": profile.min_turning_radius_m,
            "execution_ready": profile.execution_ready,
        },
        run_id=args.run_id,
    )

    cleanup = None
    if args.planner == "ours":
        if args.graph is None:
            graph_ref = scenario.metadata.get("graph_fixture")
            if not graph_ref:
                raise SystemExit("ours requires --graph or scenario graph_fixture")
            args.graph = args.scenario.parent / str(graph_ref)
        adapter = ProposedAdapter(load_agricultural_graph(args.graph))
    elif args.planner == "fields2cover":
        if args.coverage_components_json is None:
            adapter = Fields2CoverAdapter()
        else:
            components = _coverage_components(args.coverage_components_json)
            adapter = Fields2CoverAdapter(lambda _spec: (components, 0.0))
    elif args.planner in ("astar", "theta_star", "hybrid_astar", "state_lattice"):
        if args.nav2_live:
            call, navigator, rclpy = _nav2_call()
            cleanup = (navigator, rclpy)
            adapter = Nav2P2PAdapter(args.planner, call)
        else:
            adapter = Nav2P2PAdapter(args.planner)
    else:
        raise SystemExit(f"unsupported planner {args.planner!r}")

    try:
        out = ExperimentRunner(args.result_root).run(spec, adapter)
        path_csv = out / "path.csv"
        if path_csv.exists():
            points = read_path_csv(path_csv)
            render_route(points, out / "figure", title=f"{scenario.scenario_id} / {args.planner}")
        print(out)
        return 0
    finally:
        if cleanup is not None:
            navigator, rclpy = cleanup
            navigator.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
