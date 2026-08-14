"""IO helpers for frozen V25-12E R6 reverse-fallback assets."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from .contracts import AssetContractError
from .reverse_fallback_admission import (
    REVERSE_FALLBACK_ADMISSION_SCHEMA,
    ReverseFallbackAdmissionItem,
    ReverseFallbackAdmissionPlan,
)


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise AssetContractError("reverse_route_asset_invalid", f"{field} must be a mapping")
    return value


def load_reverse_fallback_admission(path: str | Path) -> ReverseFallbackAdmissionPlan:
    """Load a frozen R6A reverse-fallback admission asset."""
    source_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
    payload = _mapping(raw, "reverse_fallback_admission")
    schema = str(payload.get("schema", ""))
    if schema != REVERSE_FALLBACK_ADMISSION_SCHEMA:
        raise AssetContractError(
            "reverse_fallback_admission_schema_mismatch",
            f"expected {REVERSE_FALLBACK_ADMISSION_SCHEMA}, got {schema or '<empty>'}",
        )

    raw_items = payload.get("connectors") or []
    if not isinstance(raw_items, list):
        raise AssetContractError(
            "reverse_fallback_admission_invalid",
            "connectors must be a list",
        )
    items: list[ReverseFallbackAdmissionItem] = []
    for index, raw_item in enumerate(raw_items):
        item = _mapping(raw_item, f"connectors[{index}]")
        items.append(
            ReverseFallbackAdmissionItem(
                connector_id=str(item["connector_id"]),
                from_aisle_id=str(item["from_aisle_id"]),
                to_aisle_id=str(item["to_aisle_id"]),
                turn_zone_id=str(item["turn_zone_id"]),
                forward_audit_status=str(item.get("forward_audit_status", "")),
                decision=str(item.get("decision", "")),
                reason=str(item.get("reason", "")),
            )
        )

    declared = payload.get("connector_count")
    if declared is not None and int(declared) != len(items):
        raise AssetContractError(
            "reverse_fallback_admission_count_mismatch",
            f"declared connector_count={declared}, parsed={len(items)}",
        )

    return ReverseFallbackAdmissionPlan(
        frame_id=str(payload.get("frame_id", "map")),
        platform_id=str(payload.get("platform_id", "")),
        platform_profile_sha256=str(payload.get("platform_profile_sha256", "")),
        items=tuple(items),
        source=dict(payload.get("source") or {}),
        schema=schema,
        status=str(payload.get("status", "DRAFT")),
    )
