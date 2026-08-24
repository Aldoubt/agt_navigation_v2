#!/usr/bin/env python3
"""Replay E2 exact vertical-boundary diagnostics from an E1 result.

E2 is diagnostic-only review evidence. It re-derives A0/A3 from the same PCD
and frozen navigation configuration recorded by the source ablation run, then
revisits only MID-dominant exact E1 blocker cells against the raw PCD. It never
mutates D2/D3 evidence, obstacle policy, Formal/Accepted Navigation Map, or map
authority.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "agt_offline_assets"))

from agt_offline_assets.height_layer_ablation import load_height_layer_evidence_bundle  # noqa: E402
from agt_offline_assets.navigation_ablation import apply_navigation_ablation_profile  # noqa: E402
from agt_offline_assets.navigation_map_derivation import (  # noqa: E402
    GroundRelativeNavigationConfig,
    derive_ground_relative_navigation_map,
)
from agt_offline_assets.pcd_io import read_pcd  # noqa: E402
from agt_offline_assets.vehicle_vertical_boundary_audit import (  # noqa: E402
    build_vehicle_vertical_boundary_audit,
)


_ABLATION_SCHEMA = "agt_navigation_ablation/v3"
_E1_SCHEMA = "agt_vehicle_sensor_evidence_root_cause/v1"


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


def _vertical_evidence_path(summary_path: Path, summary: dict[str, object]) -> Path:
    control = summary.get("control")
    if not isinstance(control, dict):
        raise ValueError("ablation summary is missing control")
    return _resolve_recorded_path(summary_path, control.get("vertical_evidence_bundle"))


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
    e1_path = args.sensor_evidence_root_cause.expanduser().resolve()
    summary = _read_json(summary_path)
    e1 = _read_json(e1_path)
    if summary.get("schema") != _ABLATION_SCHEMA:
        raise ValueError(f"E2 requires {_ABLATION_SCHEMA}")
    if e1.get("schema") != _E1_SCHEMA:
        raise ValueError(f"E2 requires {_E1_SCHEMA}")

    input_pcd = _resolve_recorded_path(summary_path, summary.get("input_pcd"))
    e1_input_pcd = Path(str(e1.get("input_pcd", ""))).expanduser().resolve()
    if e1_input_pcd != input_pcd:
        raise ValueError("E2 source PCD mismatch between ablation summary and E1")

    config = _navigation_config(summary)
    evidence_path = _vertical_evidence_path(summary_path, summary)
    evidence = load_height_layer_evidence_bundle(evidence_path)
    vehicle = e1.get("vehicle_envelope")
    if not isinstance(vehicle, dict):
        raise ValueError("E2 E1 report is missing vehicle_envelope")
    collision_z_max = float(vehicle.get("collision_z_max_m"))
    if not (
        float(evidence.low_max_height_m)
        < collision_z_max
        < float(evidence.mid_max_height_m)
    ):
        raise ValueError("E2 vehicle collision_z_max must lie strictly inside MID layer")

    cloud = read_pcd(input_pcd)
    a0 = derive_ground_relative_navigation_map(cloud, config)
    a3 = apply_navigation_ablation_profile(a0, "A3")
    document = build_vehicle_vertical_boundary_audit(
        cloud,
        a3,
        e1,
        obstacle_min_height_m=float(evidence.obstacle_min_height_m),
        low_max_height_m=float(evidence.low_max_height_m),
        mid_max_height_m=float(evidence.mid_max_height_m),
        obstacle_max_height_m=float(evidence.obstacle_max_height_m),
        mid_bin_size_m=float(args.mid_bin_size_m),
        chunk_size=int(args.chunk_size),
    )
    document.update(
        {
            "source_ablation_summary": str(summary_path),
            "source_sensor_evidence_root_cause": str(e1_path),
            "input_pcd": str(input_pcd),
            "vertical_evidence_bundle": str(evidence_path),
            "replay_contract": "SAME_A0_CONFIG_PLUS_RAW_PCD_EXACT_BLOCKER_HEIGHTS",
        }
    )
    _write_report(args.output, document)
    return document


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay E2 exact vertical-boundary evidence for MID-dominant E1 blockers; "
            "diagnostic-only and never Navigation Map authority."
        )
    )
    parser.add_argument(
        "--ablation-summary",
        type=Path,
        required=True,
        help="D3.2 run root ablation_summary.json",
    )
    parser.add_argument(
        "--sensor-evidence-root-cause",
        type=Path,
        required=True,
        help="E1 vehicle_review/sensor_evidence_root_cause.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="destination vehicle_review/vertical_boundary_resolution.json",
    )
    parser.add_argument(
        "--mid-bin-size-m",
        type=float,
        default=0.01,
        help="MID-layer diagnostic histogram bin size in metres (default 0.01)",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=1_000_000,
        help="raw PCD scan chunk size",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    document = run(args)
    print(json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
