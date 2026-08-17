#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import hashlib
import math
import sys
from pathlib import Path

from agt_route_benchmark.adapters.fields2cover import Fields2CoverAdapter
from agt_route_benchmark.adapters.manual_waypoints import ManualWaypointAdapter
from agt_route_benchmark.adapters.nav2_p2p import Nav2P2PAdapter
from agt_route_benchmark.adapters.proposed import ProposedAdapter
from agt_route_benchmark.adapters.v25_route_asset import V25RouteAssetAdapter
from agt_route_benchmark.contracts import ExperimentSpec, PathPoint
from agt_route_benchmark.coverage_bridge import collect_coverage_components_live
from agt_route_benchmark.experiment import ExperimentRunner
from agt_route_benchmark.graph_io import load_agricultural_graph
from agt_route_benchmark.manual_waypoint_io import load_manual_waypoint_plan
from agt_route_benchmark.path_io import read_path_csv
from agt_route_benchmark.map_io import load_nav2_map
from agt_route_benchmark.preflight import evaluate_p2p_preflight
from agt_route_benchmark.profile import load_platform_profile
from agt_route_benchmark.renderer import render_route
from agt_route_benchmark.scenario import load_scenario
from agt_route_benchmark.site_snapshot import load_site_snapshot
from agt_route_benchmark.validation import evaluate_normalized_path


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _benchmark_cli_args(argv: list[str]) -> list[str]:
    try:
        from rclpy.utilities import remove_ros_args
        return list(remove_ros_args(args=argv)[1:])
    except ImportError:
        if "--ros-args" in argv:
            return list(argv[1:argv.index("--ros-args")])
        return list(argv[1:])


def _nav2_call(server_wait_s: float = 20.0, result_wait_s: float = 30.0):
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import ComputePathToPose
    except ImportError as exc:
        raise RuntimeError("Nav2 live mode requires ROS2 Humble + nav2_msgs") from exc

    rclpy.init()
    node = Node("agt_route_benchmark_nav2_client")
    client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")

    def pose(value):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x = value[0]
        msg.pose.position.y = value[1]
        msg.pose.orientation.z = math.sin(value[2] / 2.0)
        msg.pose.orientation.w = math.cos(value[2] / 2.0)
        return msg

    def call(plugin_id, start, goal):
        if not client.wait_for_server(timeout_sec=server_wait_s):
            raise RuntimeError(f"compute_path_to_pose action unavailable after {server_wait_s:.1f}s")
        request = ComputePathToPose.Goal()
        request.start = pose(start)
        request.goal = pose(goal)
        request.use_start = True
        request.planner_id = plugin_id
        send_future = client.send_goal_async(request)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=result_wait_s)
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError("compute_path_to_pose goal request timed out")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"planner {plugin_id} rejected compute_path_to_pose goal")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=result_wait_s)
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("compute_path_to_pose result timed out")
        response = result_future.result().result
        path = response.path
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

    return call, node, rclpy


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
    parser.add_argument("--site-snapshot", type=Path)
    parser.add_argument("--map-yaml", type=Path, help="Optional Nav2 map YAML for deterministic route-background rendering and path validation")
    parser.add_argument("--nav2-live", action="store_true")
    parser.add_argument("--graph", type=Path, help="Development-only sparse graph fixture for ours")
    parser.add_argument("--ours-route-csv", type=Path, help="Canonical V2.5 Route Asset CSV for the formal proposed method")
    parser.add_argument("--manual-waypoints-yaml", type=Path, help="Auditable manual waypoint mission plan with frozen best-P2P planner")
    parser.add_argument("--coverage-components-json", type=Path, help="Development-only frozen coverage components")
    parser.add_argument("--coverage-live", action="store_true")
    parser.add_argument("--semantic-map", type=Path)
    args = parser.parse_args(_benchmark_cli_args(sys.argv))

    scenario = load_scenario(args.scenario, formal=args.formal)
    profile = load_platform_profile(args.platform_profile)
    snapshot = None
    if args.site_snapshot is not None:
        snapshot = load_site_snapshot(args.site_snapshot, verify_assets=True)
        if snapshot["site_id"] != args.site:
            raise SystemExit("site snapshot site_id does not match --site")
        selected_profile_sha = hashlib.sha256(args.platform_profile.read_bytes()).hexdigest()
        if selected_profile_sha != snapshot["assets"]["platform_profile"]["sha256"]:
            raise SystemExit("selected platform profile is not the profile bound by site snapshot")
    elif args.formal:
        raise SystemExit("formal mode requires --site-snapshot")
    if not profile.preview_planning_enabled:
        raise SystemExit("platform profile disables preview planning")

    render_map_yaml = args.map_yaml
    if render_map_yaml is None and snapshot is not None:
        render_map_yaml = Path(snapshot["assets"]["map_yaml"]["path"])
    nav_map = load_nav2_map(render_map_yaml) if render_map_yaml is not None else None
    preflight_result = None
    if scenario.level == "p2p" and nav_map is not None:
        preflight_result = evaluate_p2p_preflight(scenario, nav_map, profile)

    evaluation_semantic_map = args.semantic_map
    if evaluation_semantic_map is None and snapshot is not None:
        evaluation_semantic_map = Path(snapshot["assets"]["semantic_map"]["path"])
    if evaluation_semantic_map is not None:
        evaluation_semantic_map = Path(evaluation_semantic_map).expanduser().resolve()
        if not evaluation_semantic_map.is_file():
            raise SystemExit(f"semantic map does not exist: {evaluation_semantic_map}")
        if snapshot is not None:
            selected_semantic_sha = hashlib.sha256(evaluation_semantic_map.read_bytes()).hexdigest()
            if selected_semantic_sha != snapshot["assets"]["semantic_map"]["sha256"]:
                raise SystemExit("selected semantic map is not the semantic map bound by site snapshot")

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
            **({
                "site_snapshot_sha256": snapshot["snapshot_sha256"],
                "site_asset_hashes": {key: value["sha256"] for key, value in snapshot["assets"].items()},
            } if snapshot is not None else {}),
        },
        run_id=args.run_id,
    )

    cleanup = None
    if args.planner == "ours":
        if args.ours_route_csv is not None:
            adapter = V25RouteAssetAdapter(args.ours_route_csv)
        elif args.formal:
            raise SystemExit(
                "formal ours requires --ours-route-csv from the accepted V2.5 maximum-feasible route pipeline; sparse graph fixtures are development-only"
            )
        else:
            if args.graph is None:
                graph_ref = scenario.metadata.get("graph_fixture")
                if not graph_ref:
                    raise SystemExit("development ours requires --graph or scenario graph_fixture")
                args.graph = args.scenario.parent / str(graph_ref)
            adapter = ProposedAdapter(load_agricultural_graph(args.graph))
    elif args.planner == "manual_waypoints_best_p2p":
        if scenario.level != "mission":
            raise SystemExit("manual_waypoints_best_p2p is a mission-level baseline")
        if args.manual_waypoints_yaml is None:
            raise SystemExit("manual_waypoints_best_p2p requires --manual-waypoints-yaml")
        waypoint_plan = load_manual_waypoint_plan(args.manual_waypoints_yaml)
        unknown_targets = sorted(set(waypoint_plan.target_semantic_ids) - set(scenario.required_semantic_ids))
        if unknown_targets:
            raise SystemExit(f"manual waypoint plan targets non-required semantic IDs: {unknown_targets}")
        if not args.nav2_live:
            raise SystemExit("manual_waypoints_best_p2p requires --nav2-live so every authored segment uses the frozen P2P planner")
        call, nav2_node, rclpy = _nav2_call()
        cleanup = (nav2_node, rclpy)
        p2p_adapter = Nav2P2PAdapter(waypoint_plan.p2p_planner, call)
        adapter = ManualWaypointAdapter(
            waypoint_plan.waypoints,
            p2p_adapter,
            visited_semantic_ids=waypoint_plan.target_semantic_ids,
        )
    elif args.planner == "fields2cover":
        if args.formal and not args.coverage_live:
            raise SystemExit("formal fields2cover requires --coverage-live; precomputed component JSON is development-only")
        semantic_map = evaluation_semantic_map
        if args.coverage_live:
            if semantic_map is None:
                raise SystemExit("--coverage-live requires --semantic-map or a bound --site-snapshot")
            adapter = Fields2CoverAdapter(
                lambda _spec: collect_coverage_components_live(semantic_map)
            )
        elif args.coverage_components_json is not None:
            components = _coverage_components(args.coverage_components_json)
            adapter = Fields2CoverAdapter(lambda _spec: (components, 0.0))
        else:
            adapter = Fields2CoverAdapter()
    elif args.planner in ("astar", "theta_star", "hybrid_astar", "state_lattice"):
        if args.formal and not args.nav2_live:
            raise SystemExit("formal P2P baselines require --nav2-live")
        if preflight_result is not None and not preflight_result.valid:
            adapter = Nav2P2PAdapter(args.planner)
        elif args.nav2_live:
            call, nav2_node, rclpy = _nav2_call()
            cleanup = (nav2_node, rclpy)
            adapter = Nav2P2PAdapter(args.planner, call)
        else:
            adapter = Nav2P2PAdapter(args.planner)
    else:
        raise SystemExit(f"unsupported planner {args.planner!r}")

    path_evaluator = None
    if nav_map is not None:
        path_evaluator = lambda points: evaluate_normalized_path(
            points,
            nav_map,
            profile,
            semantic_map_path=evaluation_semantic_map,
        )

    try:
        out = ExperimentRunner(args.result_root).run(
            spec,
            adapter,
            path_evaluator=path_evaluator,
            preflight_result=preflight_result,
        )
        path_csv = out / "path.csv"
        if path_csv.exists():
            points = read_path_csv(path_csv)
            if nav_map is not None:
                render_route(
                    points,
                    out / "figure",
                    title=f"{scenario.scenario_id} / {args.planner}",
                    map_extent=nav_map.extent,
                    occupancy_image=nav_map.image,
                    image_origin="upper",
                )
            else:
                render_route(points, out / "figure", title=f"{scenario.scenario_id} / {args.planner}")
        print(out)
        return 0
    finally:
        if cleanup is not None:
            nav2_node, rclpy = cleanup
            nav2_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
