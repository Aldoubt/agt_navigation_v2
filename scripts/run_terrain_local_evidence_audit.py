#!/usr/bin/env python3
"""Run raw-PCD Terrain Local Evidence Audit for final E3-EXACT terrain blockers.

This runner replays only the frozen A0 -> A3 terrain evidence needed to inspect
terrain blockers already identified by the E3-EXACT clearance-throat and B0
reports. It does not change slope/step thresholds, does not mutate any
Navigation Map, and never writes Formal/Accepted PGM assets.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "agt_offline_assets"))

from agt_offline_assets import (  # noqa: E402
    GroundRelativeNavigationConfig,
    derive_ground_relative_navigation_map,
    read_pcd,
)
from agt_offline_assets.navigation_ablation import (  # noqa: E402
    apply_navigation_ablation_profile,
)
from agt_offline_assets.terrain_local_evidence import (  # noqa: E402
    build_terrain_local_evidence_audit,
    select_terrain_review_targets,
)


_ABLATION_SCHEMA = "agt_navigation_ablation/v3"
_THROAT_SCHEMAS = {
    "agt_vehicle_clearance_throat_audit/v1",
    "agt_vehicle_clearance_throat_audit/v2",
}
_BLOCKER_SCHEMA = "agt_aisle_blocker_audit/v1"
_PROFILE = "E3-EXACT"


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
    throat_path = args.exact_clearance_throats.expanduser().resolve()
    blocker_path = args.exact_disconnected_root_cause.expanduser().resolve()

    summary = _read_json(summary_path)
    if summary.get("schema") != _ABLATION_SCHEMA:
        raise ValueError(f"terrain local evidence requires {_ABLATION_SCHEMA}")

    throat_audit = _read_json(throat_path)
    if str(throat_audit.get("schema")) not in _THROAT_SCHEMAS:
        raise ValueError("terrain local evidence requires a clearance-throat audit")

    blocker_audit = _read_json(blocker_path)
    if blocker_audit.get("schema") != _BLOCKER_SCHEMA:
        raise ValueError(f"terrain local evidence requires {_BLOCKER_SCHEMA}")
    recorded_profile = str(blocker_audit.get("profile", _PROFILE))
    if recorded_profile != _PROFILE:
        raise ValueError(f"terrain local evidence requires {_PROFILE} blocker evidence")

    input_pcd = _resolve_recorded_path(summary_path, summary.get("input_pcd"))
    config = _navigation_config(summary)

    cloud = read_pcd(input_pcd)
    a0 = derive_ground_relative_navigation_map(cloud, config)
    a3 = apply_navigation_ablation_profile(a0, "A3")

    targets = select_terrain_review_targets(throat_audit, blocker_audit)
    if not targets:
        raise RuntimeError("E3-EXACT final blocker reports contain no terrain review targets")

    audit = build_terrain_local_evidence_audit(
        cloud,
        a3,
        targets,
        local_half_window_m=float(args.local_half_window_m),
        lower_quantile=float(args.lower_quantile),
        minimum_points_per_cell=int(args.minimum_points_per_cell),
        minimum_surface_cells=int(args.minimum_surface_cells),
        chunk_size=int(args.chunk_size),
    )
    document: dict[str, object] = {
        **audit,
        "profile": _PROFILE,
        "input_pcd": str(input_pcd),
        "source_ablation_summary": str(summary_path),
        "source_exact_clearance_throats": str(throat_path),
        "source_exact_disconnected_root_cause": str(blocker_path),
        "target_aisle_ids": sorted({str(item["aisle_id"]) for item in targets}),
    }
    _write_report(args.output, document)
    return document


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Corroborate final E3-EXACT terrain blockers against an independent "
            "raw-PCD local lower-envelope; diagnostic only."
        )
    )
    parser.add_argument(
        "--ablation-summary",
        type=Path,
        required=True,
        help="D3.2/E3 run-root ablation_summary.json containing frozen A0 config and input PCD",
    )
    parser.add_argument(
        "--exact-clearance-throats",
        type=Path,
        required=True,
        help="E3-EXACT vehicle_review/exact_clearance_throats.json",
    )
    parser.add_argument(
        "--exact-disconnected-root-cause",
        type=Path,
        required=True,
        help="E3-EXACT vehicle_review/exact_disconnected_aisle_root_cause.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="destination vehicle_review/terrain_local_evidence.json",
    )
    parser.add_argument("--local-half-window-m", type=float, default=0.35)
    parser.add_argument("--lower-quantile", type=float, default=0.10)
    parser.add_argument("--minimum-points-per-cell", type=int, default=3)
    parser.add_argument("--minimum-surface-cells", type=int, default=9)
    parser.add_argument("--chunk-size", type=int, default=1_000_000)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    document = run(args)
    print(json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
