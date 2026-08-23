#!/usr/bin/env python3
"""Run E3 full-grid coarse-vs-exact vehicle collision ablation.

E3 consumes one existing D3.2 ablation bundle. It freezes the recorded A0
configuration, A3 terrain, row/corridor geometry, Site Boundary and vehicle
envelope, then compares:

- E3-COARSE: current D3 whole-overlapping-layer collision evidence;
- E3-EXACT: raw-PCD continuous ground-relative collision evidence.

The result is EXPERIMENTAL_REVIEW_EVIDENCE only. Neither branch is Navigation
Map authority and neither mutates Formal/Accepted PGM assets.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "agt_offline_assets"))
sys.path.insert(0, str(ROOT / "src" / "agt_ui_bridge"))
sys.path.insert(0, str(ROOT / "src" / "agt_coverage_planning"))

from agt_offline_assets import (  # noqa: E402
    CorridorRefinementConfig,
    GroundRelativeNavigationConfig,
    NavigationStructureConfig,
    derive_corridor_refinement,
    derive_ground_relative_navigation_map,
    derive_navigation_structure,
    load_site_boundary,
    materialize_structure_aware_navigation_map,
    read_pcd,
    replay_formal_navigation_overrides,
)
from agt_offline_assets.formal_navigation_map import (  # noqa: E402
    StructureAwareNavigationConfig,
    derive_hard_occupancy_provenance,
)
from agt_offline_assets.height_layer_ablation import (  # noqa: E402
    load_height_layer_evidence_bundle,
)
from agt_offline_assets.navigation_ablation import (  # noqa: E402
    apply_navigation_ablation_profile,
)
from agt_offline_assets.vehicle_collision_envelope import (  # noqa: E402
    VehicleCollisionEnvelope,
    derive_vehicle_envelope_navigation,
    select_overlapping_height_layers,
)
from agt_offline_assets.vehicle_exact_collision_ablation import (  # noqa: E402
    E3_EXACT_CONTRACT,
    derive_exact_vehicle_envelope_navigation,
    exact_vehicle_height_interval_m,
)
from agt_offline_assets.vehicle_feasibility import (  # noqa: E402
    build_vehicle_feasible_aisle_audit,
)


_SCHEMA = "agt_vehicle_exact_collision_ablation/v1"
_ABLATION_SCHEMA = "agt_navigation_ablation/v3"
_AUTHORITY = "DERIVED_VEHICLE_REVIEW_NOT_NAVIGATION_MAP_AUTHORITY"
_COARSE_CONTRACT = "WHOLE_OVERLAPPING_D2_LAYERS"
_CONNECTIVITY_SCOPE = "INTERIOR_TERMINAL_BANDS"


def _read_json(path: Path) -> dict[str, object]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"expected JSON object: {path}")
    return document


def _resolve_recorded_path(summary_path: Path, recorded: object) -> Path:
    value = str(recorded or "").strip()
    if not value:
        raise ValueError("recorded path is missing")
    path = Path(value).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (summary_path.parent / path).resolve()


def _navigation_config(summary: dict[str, object]) -> GroundRelativeNavigationConfig:
    control = summary.get("control")
    if not isinstance(control, dict):
        raise ValueError("ablation summary is missing control")
    raw = control.get("navigation_config_A0")
    if not isinstance(raw, dict):
        raise ValueError("ablation summary is missing control.navigation_config_A0")
    try:
        config = GroundRelativeNavigationConfig(**raw)
    except TypeError as exc:
        raise ValueError("invalid control.navigation_config_A0") from exc
    config.validate()
    return config


def _vehicle_envelope(summary: dict[str, object]) -> VehicleCollisionEnvelope:
    control = summary.get("control")
    if not isinstance(control, dict):
        raise ValueError("ablation summary is missing control")
    raw = control.get("vehicle_collision_envelope")
    if not isinstance(raw, dict):
        raise ValueError("E3 requires recorded control.vehicle_collision_envelope")
    required = (
        "half_width_m",
        "lateral_safety_margin_m",
        "collision_z_min_m",
        "collision_z_max_m",
    )
    if any(key not in raw for key in required):
        raise ValueError("recorded vehicle collision envelope is incomplete")
    return VehicleCollisionEnvelope(
        half_width_m=float(raw["half_width_m"]),
        lateral_safety_margin_m=float(raw["lateral_safety_margin_m"]),
        collision_z_min_m=float(raw["collision_z_min_m"]),
        collision_z_max_m=float(raw["collision_z_max_m"]),
    )


def _vertical_evidence_path(summary_path: Path, summary: dict[str, object]) -> Path:
    control = summary.get("control")
    if not isinstance(control, dict):
        raise ValueError("ablation summary is missing control")
    return _resolve_recorded_path(summary_path, control.get("vertical_evidence_bundle"))


def _profile_metrics(
    navigation,
    accepted,
    provenance,
    reports: list[dict[str, object]],
) -> dict[str, object]:
    clearances = [
        float(item.get("maximum_end_to_end_clearance_radius_m", 0.0))
        for item in reports
    ]
    return {
        "ground_navigation": navigation.counts(),
        "formal_accepted": accepted.navigation.counts(),
        "selected_obstacle_point_count": int(
            np.sum(np.asarray(navigation.obstacle_count, dtype=np.int64))
        ),
        "direct_obstacle_cell_count": int(
            np.count_nonzero(
                np.asarray(navigation.obstacle_count, dtype=np.int32)
                >= int(navigation.config.minimum_obstacle_points)
            )
        ),
        "strong_sensor_cell_count": int(
            np.count_nonzero(np.asarray(provenance.strong_sensor_obstacle_mask, dtype=bool))
        ),
        "interior_terminal_raster_connected_aisles": sum(
            bool(item.get("interior_terminal_raster_connectivity")) for item in reports
        ),
        "vehicle_feasible_connected_aisles": sum(
            bool(item.get("vehicle_feasible_connectivity")) for item in reports
        ),
        "maximum_of_end_to_end_clearance_radius_m": float(max(clearances, default=0.0)),
        "aisles": reports,
    }


def _compare_aisle_reports(
    coarse_reports: list[dict[str, object]],
    exact_reports: list[dict[str, object]],
) -> list[dict[str, object]]:
    coarse_by_id = {str(item.get("aisle_id")): item for item in coarse_reports}
    exact_by_id = {str(item.get("aisle_id")): item for item in exact_reports}
    if set(coarse_by_id) != set(exact_by_id):
        raise ValueError("E3 coarse/exact aisle sets differ")

    result: list[dict[str, object]] = []
    for aisle_id in sorted(coarse_by_id):
        coarse = coarse_by_id[aisle_id]
        exact = exact_by_id[aisle_id]
        coarse_clearance = float(coarse.get("maximum_end_to_end_clearance_radius_m", 0.0))
        exact_clearance = float(exact.get("maximum_end_to_end_clearance_radius_m", 0.0))
        result.append(
            {
                "aisle_id": aisle_id,
                "coarse_interior_terminal_raster_connectivity": bool(
                    coarse.get("interior_terminal_raster_connectivity")
                ),
                "exact_interior_terminal_raster_connectivity": bool(
                    exact.get("interior_terminal_raster_connectivity")
                ),
                "coarse_vehicle_feasible_connectivity": bool(
                    coarse.get("vehicle_feasible_connectivity")
                ),
                "exact_vehicle_feasible_connectivity": bool(
                    exact.get("vehicle_feasible_connectivity")
                ),
                "coarse_end_to_end_clearance_radius_m": coarse_clearance,
                "exact_end_to_end_clearance_radius_m": exact_clearance,
                "clearance_radius_delta_m": float(
                    round(exact_clearance - coarse_clearance, 12)
                ),
            }
        )
    return result


def _materialize_and_evaluate(
    navigation,
    *,
    structure,
    corridor,
    boundary,
    policy: StructureAwareNavigationConfig,
    envelope: VehicleCollisionEnvelope,
    terminal_inset_m: float,
):
    materialized = materialize_structure_aware_navigation_map(
        navigation,
        corridor,
        boundary,
        frame_id="map",
        row_direction_xy=structure.row_model.direction_xy,
        config=policy,
    )
    accepted = replay_formal_navigation_overrides(
        materialized.navigation,
        corridor,
        boundary,
        [],
        frame_id="map",
    )
    provenance = derive_hard_occupancy_provenance(navigation, policy)
    reports = build_vehicle_feasible_aisle_audit(
        navigation,
        structure,
        corridor,
        accepted.navigation.occupancy,
        clearance_radius_m=float(envelope.effective_lateral_radius_m),
        terminal_inset_m=float(terminal_inset_m),
    )
    return materialized, accepted, provenance, reports


def _write_report(path: Path, document: dict[str, object]) -> Path:
    output = path.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return output


def run(args) -> dict[str, object]:
    summary_path = args.ablation_summary.expanduser().resolve()
    summary = _read_json(summary_path)
    if summary.get("schema") != _ABLATION_SCHEMA:
        raise ValueError(f"E3 requires {_ABLATION_SCHEMA}")

    input_pcd = _resolve_recorded_path(summary_path, summary.get("input_pcd"))
    site_boundary = _resolve_recorded_path(summary_path, summary.get("site_boundary"))
    evidence_path = _vertical_evidence_path(summary_path, summary)
    config = _navigation_config(summary)
    envelope = _vehicle_envelope(summary)
    control = summary.get("control")
    assert isinstance(control, dict)
    terminal_inset_m = float(control.get("vehicle_terminal_inset_m", 0.50))

    cloud = read_pcd(input_pcd)
    a0 = derive_ground_relative_navigation_map(cloud, config)
    structure = derive_navigation_structure(a0, NavigationStructureConfig())
    corridor = derive_corridor_refinement(a0, structure, CorridorRefinementConfig())
    boundary = load_site_boundary(site_boundary, expected_frame_id="map")
    a3 = apply_navigation_ablation_profile(a0, "A3")

    evidence = load_height_layer_evidence_bundle(evidence_path)
    evidence.validate(expected_shape=tuple(np.asarray(a3.occupancy).shape))
    d2_full = evidence.selected_count("D2-FULL")
    if not np.array_equal(d2_full, np.asarray(a3.obstacle_count, dtype=np.int32)):
        raise RuntimeError("E3 vertical evidence no longer reproduces A3 obstacle_count")

    policy = StructureAwareNavigationConfig(
        soft_obstacle_max_count=int(args.soft_obstacle_max_count),
        soft_obstacle_max_ratio=float(args.soft_obstacle_max_ratio),
        soft_recovery_max_gap_m=float(args.soft_recovery_max_gap_m),
    )
    coarse_navigation = derive_vehicle_envelope_navigation(a3, evidence, envelope)
    exact_navigation = derive_exact_vehicle_envelope_navigation(
        a3,
        cloud,
        envelope,
        chunk_size=int(args.chunk_size),
    )

    _, coarse_accepted, coarse_provenance, coarse_reports = _materialize_and_evaluate(
        coarse_navigation,
        structure=structure,
        corridor=corridor,
        boundary=boundary,
        policy=policy,
        envelope=envelope,
        terminal_inset_m=terminal_inset_m,
    )
    _, exact_accepted, exact_provenance, exact_reports = _materialize_and_evaluate(
        exact_navigation,
        structure=structure,
        corridor=corridor,
        boundary=boundary,
        policy=policy,
        envelope=envelope,
        terminal_inset_m=terminal_inset_m,
    )

    coarse_metrics = _profile_metrics(
        coarse_navigation,
        coarse_accepted,
        coarse_provenance,
        coarse_reports,
    )
    exact_metrics = _profile_metrics(
        exact_navigation,
        exact_accepted,
        exact_provenance,
        exact_reports,
    )
    aisle_comparison = _compare_aisle_reports(coarse_reports, exact_reports)
    newly_feasible = [
        str(item["aisle_id"])
        for item in aisle_comparison
        if not bool(item["coarse_vehicle_feasible_connectivity"])
        and bool(item["exact_vehicle_feasible_connectivity"])
    ]
    lost_feasible = [
        str(item["aisle_id"])
        for item in aisle_comparison
        if bool(item["coarse_vehicle_feasible_connectivity"])
        and not bool(item["exact_vehicle_feasible_connectivity"])
    ]
    lower, upper = exact_vehicle_height_interval_m(a3, envelope)
    document: dict[str, object] = {
        "schema": _SCHEMA,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "authority": _AUTHORITY,
        "source_ablation_summary": str(summary_path),
        "input_pcd": str(input_pcd),
        "site_boundary": str(site_boundary),
        "vertical_evidence_bundle": str(evidence_path),
        "vehicle_envelope": envelope.to_dict(),
        "vehicle_terminal_inset_m": terminal_inset_m,
        "connectivity_scope": _CONNECTIVITY_SCOPE,
        "formal_policy": {
            "soft_obstacle_max_count": int(args.soft_obstacle_max_count),
            "soft_obstacle_max_ratio": float(args.soft_obstacle_max_ratio),
            "soft_recovery_max_gap_m": float(args.soft_recovery_max_gap_m),
        },
        "E3-COARSE": {
            "collision_evidence_contract": _COARSE_CONTRACT,
            "selected_vertical_layers": list(
                select_overlapping_height_layers(evidence, envelope)
            ),
            **coarse_metrics,
        },
        "E3-EXACT": {
            "collision_evidence_contract": E3_EXACT_CONTRACT,
            "exact_ground_relative_height_interval_m": [float(lower), float(upper)],
            "upper_interval_semantics": "EXCLUSIVE_UNLESS_ENVIRONMENTAL_MAX_ENDPOINT",
            **exact_metrics,
        },
        "comparison": {
            "selected_obstacle_point_delta": int(
                exact_metrics["selected_obstacle_point_count"]
            )
            - int(coarse_metrics["selected_obstacle_point_count"]),
            "strong_sensor_cell_delta": int(exact_metrics["strong_sensor_cell_count"])
            - int(coarse_metrics["strong_sensor_cell_count"]),
            "interior_terminal_raster_connected_aisle_delta": int(
                exact_metrics["interior_terminal_raster_connected_aisles"]
            )
            - int(coarse_metrics["interior_terminal_raster_connected_aisles"]),
            "vehicle_feasible_connected_aisle_delta": int(
                exact_metrics["vehicle_feasible_connected_aisles"]
            )
            - int(coarse_metrics["vehicle_feasible_connected_aisles"]),
            "newly_vehicle_feasible_aisles": newly_feasible,
            "lost_vehicle_feasible_aisles": lost_feasible,
            "aisles": aisle_comparison,
        },
    }
    _write_report(args.output, document)
    return document


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run E3 full-grid coarse-vs-exact vehicle collision evidence replay; "
            "diagnostic only, never Navigation Map authority."
        )
    )
    parser.add_argument(
        "--ablation-summary",
        type=Path,
        required=True,
        help="D3.2 run root ablation_summary.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="destination vehicle_review/exact_vehicle_collision_ablation.json",
    )
    parser.add_argument("--soft-obstacle-max-count", type=int, default=4)
    parser.add_argument("--soft-obstacle-max-ratio", type=float, default=0.05)
    parser.add_argument("--soft-recovery-max-gap-m", type=float, default=0.60)
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=1_000_000,
        help="raw PCD exact-collision scan chunk size",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    document = run(args)
    print(json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
