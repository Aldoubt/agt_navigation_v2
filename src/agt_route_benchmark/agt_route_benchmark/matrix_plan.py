from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shlex
from typing import Iterable

from .batch import canonical_expected_cells


SCENARIO_FILE_BY_ID = {
    "S01_straight_row": "S01_straight_row.yaml",
    "S02_90deg_entry": "S02_90deg_entry.yaml",
    "S03_headland_uturn": "S03_headland_uturn.yaml",
    "S04_narrow_headland": "S04_narrow_headland.yaml",
    "S05_blocked_row": "S05_blocked_row.yaml",
    "S06_full_mission": "S06_full_mission.yaml",
}


@dataclass(frozen=True)
class ExecutionCell:
    scenario_id: str
    planner_id: str
    runtime_kind: str
    status: str
    missing_inputs: tuple[str, ...]
    command: str
    result_dir: str | None = None


def _path_value(path: Path | str | None) -> Path | None:
    if path is None:
        return None
    return Path(path).expanduser().resolve()


def _existing_result(result_root: Path, site_id: str, scenario_id: str, planner_id: str) -> Path | None:
    base = result_root / site_id / scenario_id
    if not base.is_dir():
        return None
    candidates = sorted(base.glob(f"{planner_id}-*"))
    for candidate in reversed(candidates):
        if (candidate / "planner_report.json").is_file() and (candidate / "metrics.json").is_file():
            return candidate
    return None


def _missing(required: Iterable[tuple[str, Path | None]]) -> tuple[str, ...]:
    return tuple(name for name, value in required if value is None or not value.is_file())


def _arg(name: str, value) -> str:
    return f"{name}:={shlex.quote(str(value))}"


def _shell(*parts) -> str:
    return " ".join(shlex.quote(str(part)) for part in parts)


def build_execution_plan(
    *,
    result_root: Path | str,
    site_id: str,
    scenario_dir: Path | str,
    map_yaml: Path | str | None,
    platform_profile: Path | str | None,
    semantic_map: Path | str | None,
    manual_waypoints: Path | str | None,
    ours_route_csv: Path | str | None,
    lattice_filepath: Path | str | None,
    site_snapshot: Path | str | None,
    formal: bool,
    run_id: str = "run_001",
) -> tuple[ExecutionCell, ...]:
    result_root = Path(result_root).expanduser().resolve()
    scenario_dir = Path(scenario_dir).expanduser().resolve()
    map_yaml = _path_value(map_yaml)
    platform_profile = _path_value(platform_profile)
    semantic_map = _path_value(semantic_map)
    manual_waypoints = _path_value(manual_waypoints)
    ours_route_csv = _path_value(ours_route_csv)
    lattice_filepath = _path_value(lattice_filepath)
    site_snapshot = _path_value(site_snapshot)

    output: list[ExecutionCell] = []
    for scenario_id, planner_id in canonical_expected_cells():
        scenario = scenario_dir / SCENARIO_FILE_BY_ID[scenario_id]
        existing = _existing_result(result_root, site_id, scenario_id, planner_id)

        if planner_id in {"astar", "theta_star", "hybrid_astar", "state_lattice"}:
            runtime_kind = "nav2_p2p"
            required = [("scenario", scenario), ("map_yaml", map_yaml), ("platform_profile", platform_profile)]
            if planner_id == "state_lattice":
                required.append(("lattice_filepath", lattice_filepath))
            if formal:
                required.append(("site_snapshot", site_snapshot))
            missing = _missing(required)
            command_parts = [
                "ros2", "launch", "agt_route_benchmark", "benchmark_run.launch.py",
                _arg("site", site_id), _arg("scenario", scenario), _arg("planner", planner_id),
            ]
            if map_yaml is not None:
                command_parts.append(_arg("map", map_yaml))
            if platform_profile is not None:
                command_parts.append(_arg("platform_profile", platform_profile))
            command_parts.extend([_arg("result_root", result_root), _arg("run_id", run_id)])
            if planner_id == "state_lattice" and lattice_filepath is not None:
                command_parts.append(_arg("lattice_filepath", lattice_filepath))
            if formal:
                command_parts.append("formal:=true")
                if site_snapshot is not None:
                    command_parts.append(_arg("site_snapshot", site_snapshot))
            command = " ".join(command_parts)
        elif planner_id == "manual_waypoints_best_p2p":
            runtime_kind = "manual_waypoints"
            required = [
                ("scenario", scenario),
                ("map_yaml", map_yaml),
                ("platform_profile", platform_profile),
                ("manual_waypoints", manual_waypoints),
            ]
            if formal:
                required.append(("site_snapshot", site_snapshot))
            missing = _missing(required)
            command_parts = [
                "ros2", "launch", "agt_route_benchmark", "manual_mission_benchmark.launch.py",
                _arg("site", site_id), _arg("scenario", scenario),
            ]
            if manual_waypoints is not None:
                command_parts.append(_arg("manual_waypoints", manual_waypoints))
            if map_yaml is not None:
                command_parts.append(_arg("map", map_yaml))
            if platform_profile is not None:
                command_parts.append(_arg("platform_profile", platform_profile))
            command_parts.extend([_arg("result_root", result_root), _arg("run_id", run_id)])
            if lattice_filepath is not None:
                command_parts.append(_arg("lattice_filepath", lattice_filepath))
            if formal:
                command_parts.append("formal:=true")
                if site_snapshot is not None:
                    command_parts.append(_arg("site_snapshot", site_snapshot))
            command = " ".join(command_parts)
        elif planner_id == "fields2cover":
            runtime_kind = "fields2cover"
            required = [
                ("scenario", scenario),
                ("map_yaml", map_yaml),
                ("platform_profile", platform_profile),
                ("semantic_map", semantic_map),
            ]
            if formal:
                required.append(("site_snapshot", site_snapshot))
            missing = _missing(required)
            command_parts = [
                "ros2", "launch", "agt_route_benchmark", "fields2cover_mission_benchmark.launch.py",
                _arg("site", site_id), _arg("scenario", scenario),
            ]
            if semantic_map is not None:
                command_parts.append(_arg("semantic_map", semantic_map))
            if map_yaml is not None:
                command_parts.append(_arg("map", map_yaml))
            if platform_profile is not None:
                command_parts.append(_arg("platform_profile", platform_profile))
            command_parts.extend([_arg("result_root", result_root), _arg("run_id", run_id)])
            if formal:
                command_parts.append("formal:=true")
                if site_snapshot is not None:
                    command_parts.append(_arg("site_snapshot", site_snapshot))
            command = " ".join(command_parts)
        else:
            runtime_kind = "v25_route_asset"
            required = [
                ("scenario", scenario),
                ("map_yaml", map_yaml),
                ("platform_profile", platform_profile),
                ("semantic_map", semantic_map),
                ("ours_route_csv", ours_route_csv),
            ]
            if formal:
                required.append(("site_snapshot", site_snapshot))
            missing = _missing(required)
            command_parts = [
                "ros2", "run", "agt_route_benchmark", "route_benchmark_run.py",
                "--site", shlex.quote(site_id), "--scenario", shlex.quote(str(scenario)),
                "--planner", "ours",
            ]
            if ours_route_csv is not None:
                command_parts.extend(["--ours-route-csv", shlex.quote(str(ours_route_csv))])
            if map_yaml is not None:
                command_parts.extend(["--map-yaml", shlex.quote(str(map_yaml))])
            if semantic_map is not None:
                command_parts.extend(["--semantic-map", shlex.quote(str(semantic_map))])
            if platform_profile is not None:
                command_parts.extend(["--platform-profile", shlex.quote(str(platform_profile))])
            command_parts.extend(["--result-root", shlex.quote(str(result_root)), "--run-id", shlex.quote(run_id)])
            if formal:
                command_parts.append("--formal")
                if site_snapshot is not None:
                    command_parts.extend(["--site-snapshot", shlex.quote(str(site_snapshot))])
            command = " ".join(command_parts)

        if existing is not None:
            status = "COMPLETE"
        elif missing:
            status = "BLOCKED_INPUT"
        else:
            status = "READY"
        output.append(
            ExecutionCell(
                scenario_id=scenario_id,
                planner_id=planner_id,
                runtime_kind=runtime_kind,
                status=status,
                missing_inputs=missing,
                command=command,
                result_dir=str(existing) if existing is not None else None,
            )
        )
    return tuple(output)
