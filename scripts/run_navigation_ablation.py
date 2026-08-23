#!/usr/bin/env python3
"""Run controlled A0-A3 and D2 height-layer map ablations on one real PCD.

Experimental control contract:
- Row/Corridor geometry is derived once from A0 and frozen.
- D2 profiles are derived from A3 and change only vertical obstacle evidence.
- D1 vehicle feasibility is a downstream review layer; it never inflates PGM.
- D0 3D barrier audit remains optional and diagnostic only.

All outputs from this script are EXPERIMENTAL_REVIEW_EVIDENCE, never formal map
authority by themselves.
"""

from __future__ import annotations

import argparse
from collections import Counter
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
    evaluate_formal_navigation_qa,
    export_structure_aware_navigation_revision,
    load_site_boundary,
    materialize_structure_aware_navigation_map,
    read_pcd,
    replay_formal_navigation_overrides,
    write_formal_navigation_qa,
)
from agt_offline_assets.aisle_blocker_audit import (  # noqa: E402
    aggregate_aisle_blocker_causes,
    build_aisle_blocker_audit,
)
from agt_offline_assets.aisle_centerlines import derive_geometric_aisle_centerlines  # noqa: E402
from agt_offline_assets.critical_barrier_3d import (  # noqa: E402
    build_critical_barrier_height_audit,
    write_critical_barrier_height_plots,
)
from agt_offline_assets.formal_navigation_map import (  # noqa: E402
    StructureAwareNavigationConfig,
    derive_hard_occupancy_provenance,
)
from agt_offline_assets.height_layer_ablation import (  # noqa: E402
    D2_PROFILE_KEYS,
    apply_height_layer_ablation_profile,
    derive_height_layer_obstacle_evidence,
    height_layer_ablation_spec,
)
from agt_offline_assets.navigation_ablation import (  # noqa: E402
    ABLATION_PROFILE_KEYS,
    apply_navigation_ablation_profile,
    navigation_ablation_spec,
)
from agt_offline_assets.vehicle_feasibility import build_vehicle_feasible_aisle_audit  # noqa: E402


ALL_PROFILE_KEYS = tuple(ABLATION_PROFILE_KEYS) + tuple(D2_PROFILE_KEYS)


def _profile_spec(profile: str) -> dict[str, object]:
    if profile in D2_PROFILE_KEYS:
        return height_layer_ablation_spec(profile).to_dict()
    return navigation_ablation_spec(profile).to_dict()


def _write_overlay(revision: Path) -> None:
    try:
        from PIL import Image
        from agt_offline_assets.navigation_grid import load_navigation_grid
    except ImportError:
        return
    grid = load_navigation_grid(revision / "accepted" / "navigation_map.yaml")
    image = np.zeros((*grid.occupancy.shape, 4), dtype=np.uint8)
    image[grid.occupancy == 254] = (245, 245, 245, 255)
    image[grid.occupancy == 205] = (245, 190, 70, 255)
    image[grid.occupancy == 0] = (35, 35, 35, 255)
    Image.fromarray(np.flipud(image), mode="RGBA").save(
        revision / "validation" / "navigation_review_overlay.png"
    )


def _failure_mode_counts(reports: list[dict[str, object]]) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for report in reports:
        counts[str(report.get("failure_mode", "UNKNOWN"))] += 1
    return dict(sorted(counts.items()))


def _write_aisle_root_cause_report(
    revision: Path,
    *,
    profile: str,
    reports: list[dict[str, object]],
) -> tuple[Path, Path]:
    validation = revision / "validation"
    validation.mkdir(parents=True, exist_ok=True)
    json_path = validation / "aisle_root_cause.json"
    txt_path = validation / "aisle_root_cause.txt"
    document = {
        "schema": "agt_aisle_blocker_audit/v1",
        "profile": profile,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "connected_aisles": sum(bool(item.get("grid_connectivity")) for item in reports),
        "disconnected_aisles": sum(not bool(item.get("grid_connectivity")) for item in reports),
        "dominant_blocker_causes": aggregate_aisle_blocker_causes(reports),
        "failure_modes": _failure_mode_counts(reports),
        "aisles": reports,
    }
    json_path.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    lines = [
        f"profile: {profile}",
        f"connected: {document['connected_aisles']}/{len(reports)}",
        f"dominant causes: {json.dumps(document['dominant_blocker_causes'], ensure_ascii=False)}",
        "",
        "aisle | connected | failure mode | min blocker cells | dominant cause",
        "----- | --------- | ------------ | ----------------- | --------------",
    ]
    for report in reports:
        lines.append(
            f"{report['aisle_id']} | {bool(report['grid_connectivity'])} | "
            f"{report['failure_mode']} | {int(report['minimum_blocker_cell_count'])} | "
            f"{report['dominant_blocker_cause']}"
        )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, txt_path


def _write_vehicle_feasibility_report(
    revision: Path,
    *,
    profile: str,
    clearance_radius_m: float,
    terminal_inset_m: float,
    reports: list[dict[str, object]],
) -> Path:
    validation = revision / "validation"
    validation.mkdir(parents=True, exist_ok=True)
    path = validation / "vehicle_feasibility.json"
    document = {
        "schema": "agt_vehicle_feasible_aisle_audit/v2",
        "profile": profile,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "clearance_contract": "ENVIRONMENT_PLUS_LATERAL_AISLE_BOUNDARY",
        "clearance_radius_m": float(clearance_radius_m),
        "terminal_inset_m": float(terminal_inset_m),
        "raster_connected_aisles": sum(
            bool(item.get("raster_grid_connectivity")) for item in reports
        ),
        "vehicle_feasible_connected_aisles": sum(
            bool(item.get("vehicle_feasible_connectivity")) for item in reports
        ),
        "aisles": reports,
    }
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return path


def _write_critical_barrier_3d_report(
    revision: Path,
    *,
    profile: str,
    result,
    write_plots: bool,
) -> Path:
    validation = revision / "validation"
    validation.mkdir(parents=True, exist_ok=True)
    path = validation / "critical_barrier_3d.json"
    document = {
        "schema": "agt_critical_barrier_3d_audit/v1",
        "profile": profile,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "critical_blocker_count": len(result.records),
        "records": result.records,
    }
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    if write_plots:
        write_critical_barrier_height_plots(
            result,
            validation / "critical_barrier_3d_plots",
        )
    return path


def _write_d2_height_layer_report(revision: Path, *, profile: str, evidence) -> Path:
    validation = revision / "validation"
    validation.mkdir(parents=True, exist_ok=True)
    path = validation / "height_layer_ablation.json"
    document = {
        "schema": "agt_height_layer_ablation/v1",
        "profile": profile,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "profile_spec": height_layer_ablation_spec(profile).to_dict(),
        "evidence": evidence.metadata(),
        "selected_obstacle_point_count": int(np.sum(evidence.selected_count(profile), dtype=np.int64)),
    }
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return path


def _profile_summary(
    profile: str,
    *,
    navigation,
    geometric,
    materialized,
    accepted,
    qa,
    provenance,
    blocker_reports: list[dict[str, object]],
    vehicle_reports: list[dict[str, object]],
    vehicle_clearance_radius_m: float,
    vehicle_terminal_inset_m: float,
    barrier_3d_records: list[dict[str, object]] | None,
    d2_evidence,
) -> dict[str, object]:
    interior = [
        dict(item)
        for item in (qa.get("aisles") or [])
        if str(item.get("pair_kind")) == "ROW_ROW"
    ]
    summary: dict[str, object] = {
        "profile": profile,
        "profile_spec": _profile_spec(profile),
        "ground_navigation": navigation.counts(),
        "hard_occupancy_provenance": provenance.counts(),
        "formal_generated": materialized.navigation.counts(),
        "formal_accepted": accepted.navigation.counts(),
        "materialization": materialized.counts(),
        "structural_safety_status": str(
            qa.get("structural_safety_status", qa.get("status", "NOT_EVALUATED"))
        ),
        "navigation_usability_status": str(
            qa.get("navigation_usability_status", "NOT_EVALUATED")
        ),
        "expected_interior_aisles": int(geometric.expected_interior_aisles),
        "geometric_interior_aisles": int(geometric.interior_geometric_aisle_count),
        "traversable_interior_aisles": len(interior),
        "connected_interior_aisles": sum(
            bool(item.get("grid_connectivity")) for item in interior
        ),
        "largest_free_component_fraction": float(
            qa.get("largest_free_component_fraction", 0.0)
        ),
        "map_unknown_fraction": float(qa.get("map_unknown_fraction", 0.0)),
        "aisle_root_cause": {
            "disconnected_aisle_count": sum(
                not bool(item.get("grid_connectivity")) for item in blocker_reports
            ),
            "dominant_blocker_causes": aggregate_aisle_blocker_causes(blocker_reports),
            "failure_modes": _failure_mode_counts(blocker_reports),
            "minimum_blocker_cells_total": sum(
                int(item.get("minimum_blocker_cell_count", 0))
                for item in blocker_reports
                if not bool(item.get("grid_connectivity"))
            ),
        },
        "vehicle_feasibility": {
            "clearance_contract": "ENVIRONMENT_PLUS_LATERAL_AISLE_BOUNDARY",
            "clearance_radius_m": float(vehicle_clearance_radius_m),
            "terminal_inset_m": float(vehicle_terminal_inset_m),
            "raster_connected_aisles": sum(
                bool(item.get("raster_grid_connectivity")) for item in vehicle_reports
            ),
            "vehicle_feasible_connected_aisles": sum(
                bool(item.get("vehicle_feasible_connectivity")) for item in vehicle_reports
            ),
            "minimum_of_end_to_end_clearance_radius_m": float(
                min(
                    (float(item.get("maximum_end_to_end_clearance_radius_m", 0.0)) for item in vehicle_reports),
                    default=0.0,
                )
            ),
            "maximum_of_end_to_end_clearance_radius_m": float(
                max(
                    (float(item.get("maximum_end_to_end_clearance_radius_m", 0.0)) for item in vehicle_reports),
                    default=0.0,
                )
            ),
        },
    }
    if barrier_3d_records is not None:
        summary["critical_barrier_3d"] = {
            "critical_blocker_count": len(barrier_3d_records),
            "classifiable_point_count": sum(
                int(item.get("classifiable_point_count", 0)) for item in barrier_3d_records
            ),
            "obstacle_evidence_point_count": sum(
                int(item.get("obstacle_evidence_point_count", 0)) for item in barrier_3d_records
            ),
        }
    if profile in D2_PROFILE_KEYS and d2_evidence is not None:
        summary["d2_height_layers"] = {
            **d2_evidence.metadata(),
            "selected_obstacle_point_count": int(
                np.sum(d2_evidence.selected_count(profile), dtype=np.int64)
            ),
        }
    return summary


def run(args) -> dict[str, object]:
    output = args.output.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"ablation output must be empty/new: {output}")
    output.mkdir(parents=True, exist_ok=True)

    if args.enable_barrier_3d_audit and len(args.profiles) != 1:
        raise ValueError(
            "--enable-barrier-3d-audit requires exactly one profile; "
            "the 80M+ point PCD should not be rescanned per profile"
        )

    cloud = read_pcd(args.pcd)
    base_config = GroundRelativeNavigationConfig(
        resolution_m=args.resolution,
        ground_quantile=args.ground_quantile,
        maximum_ground_fill_distance_m=args.ground_fill_distance,
        ground_smoothing_radius_cells=args.ground_smoothing_radius,
        obstacle_min_height_m=args.obstacle_min_height,
        obstacle_max_height_m=args.obstacle_max_height,
        minimum_obstacle_points=args.minimum_obstacle_points,
        maximum_slope_deg=args.maximum_slope_deg,
        maximum_step_m=args.maximum_step_m,
        obstacle_padding_m=args.obstacle_padding_m,
    )
    base_config.validate()
    a0 = derive_ground_relative_navigation_map(cloud, base_config)

    frozen_structure = derive_navigation_structure(a0, NavigationStructureConfig())
    frozen_corridor = derive_corridor_refinement(
        a0, frozen_structure, CorridorRefinementConfig()
    )
    frozen_geometric = derive_geometric_aisle_centerlines(
        a0, frozen_structure, frozen_corridor
    )
    boundary = load_site_boundary(args.site_boundary, expected_frame_id="map")
    formal_policy = StructureAwareNavigationConfig(
        soft_obstacle_max_count=args.soft_obstacle_max_count,
        soft_obstacle_max_ratio=args.soft_obstacle_max_ratio,
        soft_recovery_max_gap_m=args.soft_recovery_max_gap_m,
    )

    d2_requested = any(profile in D2_PROFILE_KEYS for profile in args.profiles)
    a3 = apply_navigation_ablation_profile(a0, "A3") if d2_requested else None
    d2_evidence = None
    if d2_requested:
        d2_evidence = derive_height_layer_obstacle_evidence(
            cloud,
            a3,
            low_max_height_m=args.d2_low_max_height_m,
            mid_max_height_m=args.d2_mid_max_height_m,
            chunk_size=args.d2_chunk_size,
        )
        full = d2_evidence.selected_count("D2-FULL")
        if not np.array_equal(full, np.asarray(a3.obstacle_count, dtype=np.int32)):
            mismatch = int(np.count_nonzero(full != np.asarray(a3.obstacle_count)))
            raise RuntimeError(
                "D2-FULL contract violated: LOW+MID+HIGH must reproduce A3 "
                f"obstacle_count exactly; mismatched_cells={mismatch}"
            )

    document: dict[str, object] = {
        "schema": "agt_navigation_ablation/v2",
        "input_pcd": str(args.pcd.expanduser().resolve()),
        "site_boundary": str(args.site_boundary.expanduser().resolve()),
        "points": int(cloud.points.shape[0]),
        "profiles": {},
        "control": {
            "structure_source_profile": "A0",
            "d2_base_profile": "A3" if d2_requested else None,
            "row_count": len(frozen_structure.row_model.centers_v_m),
            "accepted_row_count": len(frozen_corridor.accepted_row_centers_v_m),
            "expected_interior_aisles": int(frozen_geometric.expected_interior_aisles),
            "geometric_interior_aisles": int(frozen_geometric.interior_geometric_aisle_count),
            "navigation_config_A0": vars(base_config),
            "vehicle_clearance_radius_m": float(args.vehicle_clearance_radius_m),
            "vehicle_terminal_inset_m": float(args.vehicle_terminal_inset_m),
            "barrier_3d_enabled": bool(args.enable_barrier_3d_audit),
            "d2_height_evidence": None if d2_evidence is None else d2_evidence.metadata(),
        },
    }

    for profile in args.profiles:
        if profile in D2_PROFILE_KEYS:
            navigation = apply_height_layer_ablation_profile(a3, d2_evidence, profile)
        else:
            navigation = apply_navigation_ablation_profile(a0, profile)

        materialized = materialize_structure_aware_navigation_map(
            navigation,
            frozen_corridor,
            boundary,
            frame_id="map",
            row_direction_xy=frozen_structure.row_model.direction_xy,
            config=formal_policy,
        )
        accepted = replay_formal_navigation_overrides(
            materialized.navigation,
            frozen_corridor,
            boundary,
            [],
            frame_id="map",
        )
        qa = evaluate_formal_navigation_qa(
            ground_evidence=navigation,
            materialized=materialized,
            accepted=accepted,
            overrides=[],
            structure=frozen_structure,
            corridor=frozen_corridor,
            site_boundary=boundary,
        )
        provenance = derive_hard_occupancy_provenance(navigation, formal_policy)
        blocker_reports = build_aisle_blocker_audit(
            navigation,
            frozen_structure,
            frozen_corridor,
            accepted.navigation.occupancy,
            provenance,
            materialized=materialized,
        )
        vehicle_reports = build_vehicle_feasible_aisle_audit(
            navigation,
            frozen_structure,
            frozen_corridor,
            accepted.navigation.occupancy,
            clearance_radius_m=args.vehicle_clearance_radius_m,
            terminal_inset_m=args.vehicle_terminal_inset_m,
        )

        barrier_3d_result = None
        if args.enable_barrier_3d_audit:
            barrier_3d_result = build_critical_barrier_height_audit(
                cloud,
                navigation,
                frozen_structure,
                frozen_corridor,
                blocker_reports,
                longitudinal_half_window_m=args.barrier_longitudinal_half_window_m,
                chunk_size=args.barrier_chunk_size,
            )

        destination = output / profile
        revision = export_structure_aware_navigation_revision(
            destination,
            ground_evidence=navigation,
            materialized=materialized,
            accepted=accepted,
            overrides=[],
            source_asset=args.pcd.name,
            frame_id="map",
            boundary_source="WORKBENCH_MANUAL_POLYGON",
        )
        write_formal_navigation_qa(
            qa, revision / "validation" / "navigation_validation.json"
        )
        _write_aisle_root_cause_report(
            revision, profile=profile, reports=blocker_reports
        )
        _write_vehicle_feasibility_report(
            revision,
            profile=profile,
            clearance_radius_m=args.vehicle_clearance_radius_m,
            terminal_inset_m=args.vehicle_terminal_inset_m,
            reports=vehicle_reports,
        )
        if profile in D2_PROFILE_KEYS:
            _write_d2_height_layer_report(
                revision, profile=profile, evidence=d2_evidence
            )
        if barrier_3d_result is not None:
            _write_critical_barrier_3d_report(
                revision,
                profile=profile,
                result=barrier_3d_result,
                write_plots=args.write_barrier_plots,
            )
        (revision / "validation" / "ablation_profile.json").write_text(
            json.dumps(
                {
                    **_profile_spec(profile),
                    "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
                    "frozen_structure_source": "A0",
                    "d2_base_profile": "A3" if profile in D2_PROFILE_KEYS else None,
                    "vehicle_clearance_radius_m": float(args.vehicle_clearance_radius_m),
                    "vehicle_terminal_inset_m": float(args.vehicle_terminal_inset_m),
                    "barrier_3d_enabled": bool(args.enable_barrier_3d_audit),
                },
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
            )
            + "\n",
            encoding="utf-8",
        )
        if args.write_overlays:
            _write_overlay(revision)

        document["profiles"][profile] = _profile_summary(
            profile,
            navigation=navigation,
            geometric=frozen_geometric,
            materialized=materialized,
            accepted=accepted,
            qa=qa,
            provenance=provenance,
            blocker_reports=blocker_reports,
            vehicle_reports=vehicle_reports,
            vehicle_clearance_radius_m=args.vehicle_clearance_radius_m,
            vehicle_terminal_inset_m=args.vehicle_terminal_inset_m,
            barrier_3d_records=(
                None if barrier_3d_result is None else barrier_3d_result.records
            ),
            d2_evidence=d2_evidence,
        )

    (output / "ablation_summary.json").write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return document


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--site-boundary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--profiles",
        nargs="+",
        choices=ALL_PROFILE_KEYS,
        default=list(ABLATION_PROFILE_KEYS),
    )
    parser.add_argument("--write-overlays", action="store_true")
    parser.add_argument("--vehicle-clearance-radius-m", type=float, default=0.30)
    parser.add_argument(
        "--vehicle-terminal-inset-m",
        type=float,
        default=0.50,
        help="D1 longitudinal inset for interior start/end terminal bands",
    )
    parser.add_argument("--enable-barrier-3d-audit", action="store_true")
    parser.add_argument("--write-barrier-plots", action="store_true")
    parser.add_argument("--barrier-longitudinal-half-window-m", type=float, default=0.40)
    parser.add_argument("--barrier-chunk-size", type=int, default=1_000_000)
    parser.add_argument("--d2-low-max-height-m", type=float, default=0.30)
    parser.add_argument("--d2-mid-max-height-m", type=float, default=0.60)
    parser.add_argument("--d2-chunk-size", type=int, default=1_000_000)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--ground-quantile", type=float, default=0.10)
    parser.add_argument("--ground-fill-distance", type=float, default=0.35)
    parser.add_argument("--ground-smoothing-radius", type=int, default=2)
    parser.add_argument("--obstacle-min-height", type=float, default=0.12)
    parser.add_argument("--obstacle-max-height", type=float, default=1.00)
    parser.add_argument("--minimum-obstacle-points", type=int, default=2)
    parser.add_argument("--maximum-slope-deg", type=float, default=18.0)
    parser.add_argument("--maximum-step-m", type=float, default=0.12)
    parser.add_argument("--obstacle-padding-m", type=float, default=0.05)
    parser.add_argument("--soft-obstacle-max-count", type=int, default=4)
    parser.add_argument("--soft-obstacle-max-ratio", type=float, default=0.05)
    parser.add_argument("--soft-recovery-max-gap-m", type=float, default=0.60)
    args = parser.parse_args(argv)
    if args.vehicle_clearance_radius_m < 0.0:
        parser.error("--vehicle-clearance-radius-m must be >= 0")
    if args.vehicle_terminal_inset_m < 0.0:
        parser.error("--vehicle-terminal-inset-m must be >= 0")
    if args.write_barrier_plots and not args.enable_barrier_3d_audit:
        parser.error("--write-barrier-plots requires --enable-barrier-3d-audit")
    document = run(args)
    print(json.dumps(document, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
