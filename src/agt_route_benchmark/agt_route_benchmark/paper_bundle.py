from __future__ import annotations

from dataclasses import dataclass
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import matplotlib.pyplot as plt
import numpy as np

from .map_io import Nav2Map, load_nav2_map
from .path_io import read_path_csv
from .synthetic_greenhouse import (
    SYNTHETIC_HEIGHT_M,
    SYNTHETIC_WIDTH_M,
    build_synthetic_greenhouse,
)


_PLANNER_LABELS = {
    "astar": "A*",
    "theta_star": "Theta*",
    "hybrid_astar": "Hybrid A*",
    "state_lattice": "State Lattice",
    "manual_waypoints_best_p2p": "Manual waypoints + best P2P",
    "fields2cover": "Fields2Cover",
    "ours": "Proposed",
}
_PREFERRED_P2P_ORDER = ("astar", "theta_star", "hybrid_astar", "state_lattice")
_DIAGNOSTIC_SCENARIOS = {
    "S02_90deg_entry": "D2_S02_planner_comparison",
    "S03_headland_uturn": "D3_S03_planner_comparison",
}
_POSE_ANNOTATIONS = {
    "requested_pose_source": "experiment_manifest.scenario_request",
    "returned_endpoint_source": "path.csv",
}


@dataclass(frozen=True)
class PaperBundleResult:
    output_dir: Path
    claims_markdown: str
    comparison_rows: tuple[Mapping[str, Any], ...]


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _discover_runs(results_root: Path) -> list[dict[str, Any]]:
    runs: list[dict[str, Any]] = []
    for metrics_path in sorted(results_root.rglob("metrics.json")):
        run_dir = metrics_path.parent
        manifest_path = run_dir / "experiment_manifest.json"
        report_path = run_dir / "planner_report.json"
        if not manifest_path.is_file() or not report_path.is_file():
            continue
        manifest = _json(manifest_path)
        report = _json(report_path)
        metrics = _json(metrics_path)
        scenario = str(manifest.get("scenario_id", "")).strip()
        planner = str(manifest.get("planner_id", report.get("planner_id", ""))).strip()
        if not scenario or not planner:
            raise ValueError(f"run lacks scenario/planner identity: {run_dir}")
        path_csv = run_dir / "path.csv"
        runs.append(
            {
                "run_dir": run_dir,
                "manifest_path": manifest_path,
                "planner_report_path": report_path,
                "metrics_path": metrics_path,
                "path_csv": path_csv if path_csv.is_file() else None,
                "manifest": manifest,
                "report": report,
                "metrics": metrics,
                "scenario_id": scenario,
                "planner_id": planner,
            }
        )
    if not runs:
        raise ValueError(f"no benchmark result runs found under {results_root}")
    return runs


def _comparison_row(run: Mapping[str, Any], results_root: Path) -> dict[str, Any]:
    metrics = dict(run["metrics"])
    report = dict(run["report"])
    manifest = dict(run["manifest"])
    path_csv = run.get("path_csv")
    row: dict[str, Any] = {
        "scenario_id": run["scenario_id"],
        "planner_id": run["planner_id"],
        "planner_label": _PLANNER_LABELS.get(run["planner_id"], run["planner_id"]),
        "run_id": str(manifest.get("run_id", "")),
        "formal": bool(manifest.get("formal", False)),
        "development_fixture": bool(manifest.get("development_fixture", False)),
        "run_dir": str(Path(run["run_dir"]).relative_to(results_root)),
        "success": bool(report.get("success", metrics.get("success", False))),
        "error_code": str(report.get("error_code", metrics.get("error_code", ""))),
        "metrics_sha256": _sha256(Path(run["metrics_path"])),
        "path_sha256": _sha256(Path(path_csv)) if path_csv is not None else None,
    }
    for key, value in sorted(metrics.items()):
        if key not in row:
            row[key] = value
    return row


def _write_comparison(rows: Sequence[Mapping[str, Any]], output_dir: Path) -> None:
    keys = [
        "scenario_id",
        "planner_id",
        "planner_label",
        "run_id",
        "formal",
        "development_fixture",
        "run_dir",
        "success",
        "error_code",
        "metrics_sha256",
        "path_sha256",
    ]
    extra = sorted({key for row in rows for key in row if key not in keys})
    fieldnames = [*keys, *extra]
    with (output_dir / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})
    (output_dir / "comparison.json").write_text(
        json.dumps(list(rows), indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _individual_claim(row: Mapping[str, Any]) -> str:
    label = str(row["planner_label"])
    error_code = str(row.get("error_code", ""))
    if error_code == "INVALID_SCENARIO":
        return (
            f"- **{label}:** No planner conclusion is drawn because this run was classified "
            "as invalid scenario input before planner evaluation."
        )
    if not bool(row.get("success", False)):
        return (
            f"- **{label}:** The planner returned no successful path for this frozen scenario "
            f"(`{error_code or 'planner_failure'}`). This statement is scenario-specific and is not "
            "a general capability claim."
        )

    collision = row.get("collision_free")
    kinematic = row.get("kinematic_feasible")
    execution = row.get("execution_feasible")
    if collision is True and kinematic is False:
        return (
            f"- **{label}:** {label} returned a collision-free geometric path, but the independent "
            "evaluator classified it as kinematically infeasible under the frozen vehicle constraints."
        )
    if execution is True:
        return (
            f"- **{label}:** {label} passed the frozen path-level execution-feasibility checks. "
            "This is path-level benchmark evidence, not real-vehicle tracking validation."
        )
    if collision is False:
        return (
            f"- **{label}:** The planner returned a path, but independent full-footprint evaluation "
            "found a collision; planner success and execution feasibility are therefore distinct."
        )
    if execution is False:
        return (
            f"- **{label}:** The planner returned a path, while the independent evaluator classified "
            "the resulting path as execution-infeasible under the frozen checks."
        )
    return (
        f"- **{label}:** The planner returned a path, but the available result does not contain enough "
        "independent feasibility fields for a stronger execution claim."
    )


def _scenario_claims(rows: Sequence[Mapping[str, Any]]) -> str:
    sections: list[str] = [
        "# Evidence-conditioned diagnostic claims",
        "",
        "These statements are generated only from stored benchmark outputs. Synthetic diagnostic runs "
        "are mechanism checks, not formal greenhouse headline results. Single-run planning times are "
        "descriptive and are not used for performance-ranking claims.",
    ]
    scenarios = sorted({str(row["scenario_id"]) for row in rows})
    for scenario in scenarios:
        group = [row for row in rows if row["scenario_id"] == scenario]
        sections.extend(["", f"## {scenario}", ""])
        for row in sorted(group, key=lambda r: (_planner_rank(str(r["planner_id"])), str(r["planner_id"]))):
            sections.append(_individual_claim(row))

        evaluated = [row for row in group if row.get("error_code") != "INVALID_SCENARIO"]
        feasible_values = [row.get("execution_feasible") for row in evaluated if row.get("execution_feasible") is not None]
        if len(evaluated) >= 2 and len(feasible_values) == len(evaluated) and len(set(feasible_values)) == 1:
            sections.extend([
                "",
                "**Scenario-level interpretation:** This scenario did not separate the evaluated planners "
                "in path-level feasibility and therefore cannot support a planner-capability difference claim.",
            ])
        elif len(evaluated) >= 2 and any(value is True for value in feasible_values) and any(value is False for value in feasible_values):
            sections.extend([
                "",
                "**Scenario-level interpretation:** The stored results separate geometric planning success "
                "from path-level execution feasibility for at least one evaluated planner. The evidence is "
                "limited to this frozen scenario and constraint set.",
            ])
    return "\n".join(sections).rstrip() + "\n"


def _planner_rank(planner: str) -> int:
    try:
        return _PREFERRED_P2P_ORDER.index(planner)
    except ValueError:
        return len(_PREFERRED_P2P_ORDER)


def _map_background(ax, nav_map: Nav2Map | None) -> None:
    if nav_map is None:
        return
    ax.imshow(
        nav_map.image,
        extent=nav_map.extent,
        origin="upper",
        cmap="gray",
        interpolation="nearest",
    )


def _save_figure(fig, stem: Path) -> None:
    for suffix in ("svg", "pdf", "png"):
        fig.savefig(stem.with_suffix(f".{suffix}"), bbox_inches="tight", dpi=220)
    plt.close(fig)


def _synthetic_problem_figure(runs: Sequence[Mapping[str, Any]], output_stem: Path) -> list[str]:
    fixture = build_synthetic_greenhouse()
    fig, ax = plt.subplots(figsize=(11.0, 6.8))
    ax.imshow(
        fixture.image,
        extent=(0.0, SYNTHETIC_WIDTH_M, 0.0, SYNTHETIC_HEIGHT_M),
        origin="upper",
        cmap="gray",
        interpolation="nearest",
    )
    for scenario_id, scenario in sorted(fixture.scenarios.items()):
        start = scenario["start"]
        goal = scenario["goal"]
        assert isinstance(start, Mapping)
        assert isinstance(goal, Mapping)
        sx, sy, syaw = float(start["x"]), float(start["y"]), float(start["yaw"])
        gx, gy, gyaw = float(goal["x"]), float(goal["y"]), float(goal["yaw"])
        ax.scatter([sx], [sy], marker="o", s=28)
        ax.scatter([gx], [gy], marker="x", s=34)
        ax.arrow(sx, sy, 0.7 * math.cos(syaw), 0.7 * math.sin(syaw), head_width=0.18, length_includes_head=True)
        ax.arrow(gx, gy, 0.7 * math.cos(gyaw), 0.7 * math.sin(gyaw), head_width=0.18, length_includes_head=True)
        ax.text((sx + gx) / 2.0, (sy + gy) / 2.0, scenario_id.split("_")[0], fontsize=8)
    ax.text(2.0, 18.6, "wide headland", fontsize=9)
    ax.text(23.0, 18.7, "narrow headland", fontsize=9)
    ax.text(12.5, 10.9, "permanent obstacle", fontsize=8)
    ax.set_title("D1 Controlled synthetic greenhouse diagnostic geometry")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_xlim(0.0, SYNTHETIC_WIDTH_M)
    ax.set_ylim(0.0, SYNTHETIC_HEIGHT_M)
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    _save_figure(fig, output_stem)
    return [str(run["run_dir"]) for run in runs]


def _metric_text(run: Mapping[str, Any]) -> str:
    metrics = run["metrics"]
    length = metrics.get("path_length_m")
    curvature = metrics.get("max_abs_curvature_1pm")
    limit = metrics.get("required_max_curvature_1pm")
    feasible = metrics.get("execution_feasible")
    goal_deviation = metrics.get("goal_pose_deviation_m")
    goal_heading_deviation = metrics.get("goal_heading_deviation_rad")

    def fmt(value):
        return "N/A" if value is None else f"{float(value):.3f}"

    feasible_text = "N/A" if feasible is None else ("PASS" if feasible else "FAIL")
    return (
        f"L={fmt(length)} m\n"
        f"κmax={fmt(curvature)} 1/m\n"
        f"κlimit={fmt(limit)} 1/m\n"
        f"Δgoal={fmt(goal_deviation)} m\n"
        f"Δψgoal={fmt(goal_heading_deviation)} rad\n"
        f"execution={feasible_text}"
    )


def _requested_pose(manifest: Mapping[str, Any], key: str) -> tuple[float, float, float] | None:
    request = manifest.get("scenario_request")
    if not isinstance(request, Mapping):
        return None
    pose = request.get(key)
    if not isinstance(pose, Mapping):
        return None
    try:
        return float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_rad"])
    except (KeyError, TypeError, ValueError):
        return None


def _draw_requested_pose(ax, pose: tuple[float, float, float] | None, *, marker: str) -> None:
    if pose is None:
        return
    x, y, yaw = pose
    ax.scatter([x], [y], marker=marker, s=28, facecolors="none", edgecolors="black", linewidths=1.0)
    arrow_length = 0.55
    ax.arrow(
        x,
        y,
        arrow_length * math.cos(yaw),
        arrow_length * math.sin(yaw),
        head_width=0.13,
        length_includes_head=True,
        linestyle="--",
        linewidth=1.0,
    )


def _comparison_figure(
    scenario: str,
    runs: Sequence[Mapping[str, Any]],
    output_stem: Path,
    nav_map: Nav2Map | None,
) -> list[str]:
    ordered = sorted(runs, key=lambda r: (_planner_rank(str(r["planner_id"])), str(r["planner_id"])))
    count = max(1, len(ordered))
    fig, axes = plt.subplots(1, count, figsize=(4.5 * count, 5.1), squeeze=False)
    for ax, run in zip(axes[0], ordered):
        _map_background(ax, nav_map)
        path_csv = run.get("path_csv")
        if path_csv is not None:
            points = read_path_csv(path_csv)
            if points:
                ax.plot([p.x_m for p in points], [p.y_m for p in points], linewidth=2.0)
                ax.scatter([points[0].x_m], [points[0].y_m], marker="o", s=35, label="returned start")
                ax.scatter([points[-1].x_m], [points[-1].y_m], marker="x", s=45, label="returned goal")
        _draw_requested_pose(ax, _requested_pose(run["manifest"], "start"), marker="o")
        _draw_requested_pose(ax, _requested_pose(run["manifest"], "goal"), marker="s")

        metrics = run["metrics"]
        if not run["report"].get("success", False):
            status = "NO PATH"
            ax.text(0.5, 0.5, "NO PATH", transform=ax.transAxes, ha="center", va="center", fontsize=14)
        else:
            status = "feasible" if metrics.get("execution_feasible") is True else (
                "infeasible" if metrics.get("execution_feasible") is False else "not evaluated"
            )
        ax.set_title(f"{_PLANNER_LABELS.get(run['planner_id'], run['planner_id'])}\n{status}")
        ax.text(
            0.02,
            0.02,
            _metric_text(run),
            transform=ax.transAxes,
            ha="left",
            va="bottom",
            fontsize=7.2,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.82},
        )
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.25)
    fig.suptitle(f"{scenario}: same-map planner comparison")
    fig.tight_layout()
    _save_figure(fig, output_stem)
    return [str(run["run_dir"]) for run in ordered]


def _feasibility_matrix(runs: Sequence[Mapping[str, Any]], output_stem: Path) -> list[str]:
    ordered = sorted(
        runs,
        key=lambda r: (str(r["scenario_id"]), _planner_rank(str(r["planner_id"])), str(r["planner_id"])),
    )
    columns = ("planner success", "collision free", "kinematic", "execution")
    values = np.full((len(ordered), len(columns)), np.nan, dtype=float)
    for row_index, run in enumerate(ordered):
        metrics = run["metrics"]
        report = run["report"]
        raw = (
            report.get("success"),
            metrics.get("collision_free"),
            metrics.get("kinematic_feasible"),
            metrics.get("execution_feasible"),
        )
        for column_index, value in enumerate(raw):
            if value is not None:
                values[row_index, column_index] = 1.0 if bool(value) else 0.0

    height = max(3.0, 0.48 * max(1, len(ordered)) + 1.3)
    fig, ax = plt.subplots(figsize=(8.2, height))
    masked = np.ma.masked_invalid(values)
    ax.imshow(masked, vmin=0.0, vmax=1.0, cmap="gray_r", aspect="auto")
    for i in range(values.shape[0]):
        for j in range(values.shape[1]):
            value = values[i, j]
            text = "N/A" if np.isnan(value) else ("PASS" if value > 0.5 else "FAIL")
            ax.text(j, i, text, ha="center", va="center", fontsize=8)
    ax.set_xticks(range(len(columns)), labels=columns)
    ax.set_yticks(
        range(len(ordered)),
        labels=[f"{run['scenario_id']} / {_PLANNER_LABELS.get(run['planner_id'], run['planner_id'])}" for run in ordered],
    )
    ax.set_title("D4 Independent feasibility evidence matrix")
    fig.tight_layout()
    _save_figure(fig, output_stem)
    return [str(run["run_dir"]) for run in ordered]


def build_paper_bundle(
    results_root: Path | str,
    output_dir: Path | str,
    *,
    map_yaml: Path | str | None = None,
) -> PaperBundleResult:
    results_root = Path(results_root).expanduser().resolve()
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    runs = _discover_runs(results_root)
    rows = tuple(_comparison_row(run, results_root) for run in runs)
    _write_comparison(rows, output_dir)

    claims = _scenario_claims(rows)
    (output_dir / "claims.md").write_text(claims, encoding="utf-8")
    nav_map = load_nav2_map(map_yaml) if map_yaml is not None else None

    figures: dict[str, dict[str, Any]] = {}
    if any(str(run["manifest"].get("site_id", "")) == "synthetic_greenhouse_v1" for run in runs):
        source_runs = _synthetic_problem_figure(runs, output_dir / "D1_synthetic_problem")
        figures["D1_synthetic_problem"] = {
            "source_runs": source_runs,
            "geometry_source": "agt_route_benchmark.synthetic_greenhouse",
            "formats": ["svg", "pdf", "png"],
        }

    for scenario, figure_id in _DIAGNOSTIC_SCENARIOS.items():
        scenario_runs = [run for run in runs if run["scenario_id"] == scenario]
        if not scenario_runs:
            continue
        source_runs = _comparison_figure(scenario, scenario_runs, output_dir / figure_id, nav_map)
        figures[figure_id] = {
            "scenario_id": scenario,
            "source_runs": source_runs,
            "metrics_used": [
                "success",
                "path_length_m",
                "max_abs_curvature_1pm",
                "required_max_curvature_1pm",
                "goal_pose_deviation_m",
                "goal_heading_deviation_rad",
                "collision_free",
                "kinematic_feasible",
                "execution_feasible",
            ],
            "pose_annotations": dict(_POSE_ANNOTATIONS),
            "formats": ["svg", "pdf", "png"],
        }

    evidence_runs = [run for run in runs if run["scenario_id"] in _DIAGNOSTIC_SCENARIOS]
    if evidence_runs:
        source_runs = _feasibility_matrix(evidence_runs, output_dir / "D4_feasibility_matrix")
        figures["D4_feasibility_matrix"] = {
            "source_runs": source_runs,
            "metrics_used": ["success", "collision_free", "kinematic_feasible", "execution_feasible"],
            "formats": ["svg", "pdf", "png"],
        }

    source_files: dict[str, str] = {}
    for run in runs:
        for key in ("manifest_path", "planner_report_path", "metrics_path", "path_csv"):
            path = run.get(key)
            if path is not None and Path(path).is_file():
                source_files[str(Path(path).relative_to(results_root))] = _sha256(Path(path))
    if map_yaml is not None:
        map_yaml_path = Path(map_yaml).expanduser().resolve()
        source_files[f"external_map/{map_yaml_path.name}"] = _sha256(map_yaml_path)
        if nav_map is not None:
            source_files[f"external_map/{nav_map.image_path.name}"] = _sha256(nav_map.image_path)

    manifest = {
        "schema_version": "1.0",
        "bundle_policy": "evidence_conditioned_no_posthoc_planner_tuning",
        "source_results_root": str(results_root),
        "source_files_sha256": dict(sorted(source_files.items())),
        "figures": figures,
        "claim_policy": {
            "synthetic_is_diagnostic_only": True,
            "single_run_runtime_ranking_allowed": False,
            "invalid_scenario_supports_planner_claim": False,
            "planner_success_distinct_from_execution_feasibility": True,
        },
    }
    (output_dir / "figure_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return PaperBundleResult(output_dir=output_dir, claims_markdown=claims, comparison_rows=rows)
