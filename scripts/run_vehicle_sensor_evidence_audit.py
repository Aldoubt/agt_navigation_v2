#!/usr/bin/env python3
"""Replay E1 exact throat sensor-evidence diagnostics from a D3.2 run.

E1 is diagnostic-only review evidence. It re-derives A0/A3 from the same PCD
and frozen navigation configuration recorded by ``run_navigation_ablation.py``,
loads the persisted D2.1 LOW/MID/HIGH sidecar, reconstructs the D3 vehicle
navigation evidence, and audits only D3.2 exact throat blockers attributed to
``STRONG_SENSOR_OBSTACLE``.

The script never mutates or promotes a Formal/Accepted Navigation Map. It also
intentionally preserves the same strict diagnostic frame assumptions as the
source ablation run; it does not repair the separate Workbench map-frame
transform authority contract.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "agt_offline_assets"))

from agt_offline_assets.formal_navigation_map import (  # noqa: E402
    StructureAwareNavigationConfig,
    derive_hard_occupancy_provenance,
)
from agt_offline_assets.height_layer_ablation import (  # noqa: E402
    load_height_layer_evidence_bundle,
)
from agt_offline_assets.navigation_ablation import apply_navigation_ablation_profile  # noqa: E402
from agt_offline_assets.navigation_map_derivation import (  # noqa: E402
    GroundRelativeNavigationConfig,
    derive_ground_relative_navigation_map,
)
from agt_offline_assets.pcd_io import read_pcd  # noqa: E402
from agt_offline_assets.vehicle_collision_envelope import (  # noqa: E402
    VehicleCollisionEnvelope,
    derive_vehicle_envelope_navigation,
    select_overlapping_height_layers,
)
from agt_offline_assets.vehicle_sensor_evidence_audit import (  # noqa: E402
    build_vehicle_sensor_evidence_root_cause_audit,
)


_ABLATION_SCHEMA = "agt_navigation_ablation/v3"
_D32_SCHEMA = "agt_vehicle_clearance_throat_audit/v2"


def _read_json(path: Path) -> dict[str, object]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"expected JSON object: {path}")
    return document


def _resolve_recorded_path(summary_path: Path, recorded: object) -> Path:
    value = str(recorded or "").strip()
    if not value:
        raise ValueError("ablation summary is missing a required recorded path")
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
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


def _vehicle_contract(
    summary: dict[str, object],
    evidence,
) -> tuple[VehicleCollisionEnvelope, tuple[str, ...]]:
    raw = summary.get("vehicle_collision_envelope")
    if not isinstance(raw, dict):
        raise ValueError("ablation summary is missing vehicle_collision_envelope")
    envelope_raw = raw.get("envelope")
    if not isinstance(envelope_raw, dict):
        raise ValueError("vehicle_collision_envelope.envelope is missing")
    try:
        envelope = VehicleCollisionEnvelope(
            half_width_m=float(envelope_raw["half_width_m"]),
            lateral_safety_margin_m=float(envelope_raw.get("lateral_safety_margin_m", 0.0)),
            collision_z_min_m=float(envelope_raw["collision_z_min_m"]),
            collision_z_max_m=float(envelope_raw["collision_z_max_m"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid vehicle collision envelope in ablation summary") from exc

    derived_layers = select_overlapping_height_layers(evidence, envelope)
    recorded_layers = raw.get("selected_vertical_layers")
    if not isinstance(recorded_layers, list):
        raise ValueError("vehicle_collision_envelope.selected_vertical_layers is missing")
    recorded = tuple(str(value).strip().upper() for value in recorded_layers)
    if recorded != derived_layers:
        raise ValueError(
            "vehicle selected-layer contract mismatch: "
            f"recorded={recorded!r} derived={derived_layers!r}"
        )
    return envelope, derived_layers


def _vertical_evidence_path(
    summary_path: Path,
    summary: dict[str, object],
) -> Path:
    control = summary.get("control")
    if not isinstance(control, dict):
        raise ValueError("ablation summary is missing control")
    recorded = control.get("vertical_evidence_bundle")
    return _resolve_recorded_path(summary_path, recorded)


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
    throat_path = args.clearance_throats.expanduser().resolve()
    summary = _read_json(summary_path)
    if summary.get("schema") != _ABLATION_SCHEMA:
        raise ValueError(f"E1 requires {_ABLATION_SCHEMA}")
    throat_audit = _read_json(throat_path)
    if throat_audit.get("schema") != _D32_SCHEMA:
        raise ValueError(f"E1 requires {_D32_SCHEMA}")

    input_pcd = _resolve_recorded_path(summary_path, summary.get("input_pcd"))
    config = _navigation_config(summary)
    evidence_path = _vertical_evidence_path(summary_path, summary)
    evidence = load_height_layer_evidence_bundle(evidence_path)
    envelope, selected_layers = _vehicle_contract(summary, evidence)

    cloud = read_pcd(input_pcd)
    a0 = derive_ground_relative_navigation_map(cloud, config)
    a3 = apply_navigation_ablation_profile(a0, "A3")
    vehicle_navigation = derive_vehicle_envelope_navigation(a3, evidence, envelope)
    formal_policy = StructureAwareNavigationConfig(
        soft_obstacle_max_count=int(args.soft_obstacle_max_count),
        soft_obstacle_max_ratio=float(args.soft_obstacle_max_ratio),
    )
    provenance = derive_hard_occupancy_provenance(vehicle_navigation, formal_policy)

    document = build_vehicle_sensor_evidence_root_cause_audit(
        vehicle_navigation,
        evidence,
        throat_audit,
        provenance,
        selected_layers=selected_layers,
        soft_obstacle_max_count=int(args.soft_obstacle_max_count),
        soft_obstacle_max_ratio=float(args.soft_obstacle_max_ratio),
    )
    document.update(
        {
            "source_ablation_summary": str(summary_path),
            "source_clearance_throats": str(throat_path),
            "input_pcd": str(input_pcd),
            "vertical_evidence_bundle": str(evidence_path),
            "vehicle_envelope": envelope.to_dict(),
            "replay_contract": "SAME_A0_CONFIG_PLUS_PERSISTED_D21_VERTICAL_EVIDENCE",
        }
    )
    _write_report(args.output, document)
    return document


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay E1 exact strong-sensor throat evidence from a D3.2 ablation run; "
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
        "--clearance-throats",
        type=Path,
        required=True,
        help="D3.2 vehicle_review/clearance_throats.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="destination sensor_evidence_root_cause.json",
    )
    parser.add_argument(
        "--soft-obstacle-max-count",
        type=int,
        default=4,
        help="same soft/strong obstacle-count review threshold used by the source run",
    )
    parser.add_argument(
        "--soft-obstacle-max-ratio",
        type=float,
        default=0.05,
        help="same soft/strong obstacle-ratio review threshold used by the source run",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    document = run(args)
    print(json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
