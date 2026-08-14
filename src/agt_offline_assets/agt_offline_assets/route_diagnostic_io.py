"""IO helpers for frozen V25-12E route diagnostics."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from .contracts import AssetContractError
from .forward_connector_diagnostics import (
    FORWARD_CONNECTOR_ZONE_FIT_SCHEMA,
    ForwardConnectorZoneFitDiagnostic,
    ForwardConnectorZoneFitReport,
)


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise AssetContractError("route_diagnostic_invalid", f"{field} must be a mapping")
    return value


def load_forward_connector_zone_fit_report(
    path: str | Path,
) -> ForwardConnectorZoneFitReport:
    """Load one frozen R5 row-frame Turn Zone deficit report."""
    source_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
    payload = _mapping(raw, "forward_connector_zone_fit")
    schema = str(payload.get("schema", ""))
    if schema != FORWARD_CONNECTOR_ZONE_FIT_SCHEMA:
        raise AssetContractError(
            "forward_zone_fit_schema_mismatch",
            f"expected {FORWARD_CONNECTOR_ZONE_FIT_SCHEMA}, got {schema or '<empty>'}",
        )

    diagnostics: list[ForwardConnectorZoneFitDiagnostic] = []
    raw_items = payload.get("diagnostics") or []
    if not isinstance(raw_items, list):
        raise AssetContractError("forward_zone_fit_invalid", "diagnostics must be a list")
    for index, raw_item in enumerate(raw_items):
        item = _mapping(raw_item, f"diagnostics[{index}]")
        diagnostics.append(
            ForwardConnectorZoneFitDiagnostic(
                connector_id=str(item["connector_id"]),
                from_aisle_id=str(item["from_aisle_id"]),
                to_aisle_id=str(item["to_aisle_id"]),
                turn_zone_id=str(item["turn_zone_id"]),
                status=str(item["status"]),
                candidate_count=int(item.get("candidate_count", 0)),
                best_path_type=(
                    None if item.get("best_path_type") is None else str(item["best_path_type"])
                ),
                best_candidate_length_m=(
                    None
                    if item.get("best_candidate_length_m") is None
                    else float(item["best_candidate_length_m"])
                ),
                inside_turn_zone_fraction=float(item.get("inside_turn_zone_fraction", 0.0)),
                required_outward_extension_m=float(item.get("required_outward_extension_m", 0.0)),
                required_inward_extension_m=float(item.get("required_inward_extension_m", 0.0)),
                required_lateral_low_extension_m=float(
                    item.get("required_lateral_low_extension_m", 0.0)
                ),
                required_lateral_high_extension_m=float(
                    item.get("required_lateral_high_extension_m", 0.0)
                ),
                max_required_extension_m=float(item.get("max_required_extension_m", 0.0)),
                reason=str(item.get("reason", "")),
            )
        )

    declared = payload.get("diagnostic_count")
    if declared is not None and int(declared) != len(diagnostics):
        raise AssetContractError(
            "forward_zone_fit_count_mismatch",
            f"declared diagnostic_count={declared}, parsed={len(diagnostics)}",
        )

    return ForwardConnectorZoneFitReport(
        frame_id=str(payload.get("frame_id", "map")),
        platform_id=str(payload.get("platform_id", "")),
        platform_profile_sha256=str(payload.get("platform_profile_sha256", "")),
        diagnostics=tuple(diagnostics),
        source=dict(payload.get("source") or {}),
        schema=schema,
        status=str(payload.get("status", "DRAFT")),
    )
