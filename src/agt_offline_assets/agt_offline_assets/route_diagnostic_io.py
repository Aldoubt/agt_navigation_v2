"""IO helpers for frozen V25-12E route diagnostics."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from .contracts import AssetContractError
from .forward_connector_candidate_audit import (
    FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA,
    ForwardCandidateAuditItem,
    ForwardConnectorCandidateAuditPlan,
    ForwardConnectorCandidateAuditResult,
)
from .forward_connector_diagnostics import (
    FORWARD_CONNECTOR_ZONE_FIT_SCHEMA,
    ForwardConnectorZoneFitDiagnostic,
    ForwardConnectorZoneFitReport,
)
from .forward_connector_navigation_gate import GridPathEvidence


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise AssetContractError("route_diagnostic_invalid", f"{field} must be a mapping")
    return value


def _grid_evidence(value: Any, field: str) -> GridPathEvidence:
    payload = _mapping(value, field)
    return GridPathEvidence(
        cell_count=int(payload.get("cell_count", 0)),
        free_count=int(payload.get("free_count", 0)),
        occupied_count=int(payload.get("occupied_count", 0)),
        unknown_count=int(payload.get("unknown_count", 0)),
        free_fraction=float(payload.get("free_fraction", 0.0)),
        occupied_fraction=float(payload.get("occupied_fraction", 0.0)),
        unknown_fraction=float(payload.get("unknown_fraction", 0.0)),
        grid_coverage_fraction=float(payload.get("grid_coverage_fraction", 0.0)),
    )


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


def load_forward_connector_candidate_audit(
    path: str | Path,
) -> ForwardConnectorCandidateAuditPlan:
    """Load one frozen R5.6 all-Dubins candidate audit."""
    source_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
    payload = _mapping(raw, "forward_connector_candidate_audit")
    schema = str(payload.get("schema", ""))
    if schema != FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA:
        raise AssetContractError(
            "forward_candidate_audit_schema_mismatch",
            f"expected {FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA}, got {schema or '<empty>'}",
        )

    raw_connectors = payload.get("connectors") or []
    if not isinstance(raw_connectors, list):
        raise AssetContractError("forward_candidate_audit_invalid", "connectors must be a list")

    connectors: list[ForwardConnectorCandidateAuditResult] = []
    for connector_index, raw_connector in enumerate(raw_connectors):
        item = _mapping(raw_connector, f"connectors[{connector_index}]")
        raw_candidates = item.get("candidates") or []
        if not isinstance(raw_candidates, list):
            raise AssetContractError(
                "forward_candidate_audit_invalid",
                f"connectors[{connector_index}].candidates must be a list",
            )
        candidates: list[ForwardCandidateAuditItem] = []
        for candidate_index, raw_candidate in enumerate(raw_candidates):
            candidate = _mapping(
                raw_candidate,
                f"connectors[{connector_index}].candidates[{candidate_index}]",
            )
            extension = _mapping(
                candidate.get("required_zone_extension") or {},
                f"connectors[{connector_index}].candidates[{candidate_index}].required_zone_extension",
            )
            candidates.append(
                ForwardCandidateAuditItem(
                    path_type=str(candidate.get("path_type", "")),
                    length_m=float(candidate.get("length_m", 0.0)),
                    required_outward_extension_m=float(extension.get("outward_m", 0.0)),
                    required_inward_extension_m=float(extension.get("inward_m", 0.0)),
                    required_lateral_low_extension_m=float(extension.get("lateral_low_m", 0.0)),
                    required_lateral_high_extension_m=float(extension.get("lateral_high_m", 0.0)),
                    max_required_zone_extension_m=float(extension.get("max_m", 0.0)),
                    centerline_evidence=_grid_evidence(
                        candidate.get("centerline_evidence") or {},
                        f"connectors[{connector_index}].candidates[{candidate_index}].centerline_evidence",
                    ),
                    footprint_evidence=_grid_evidence(
                        candidate.get("preview_footprint_evidence") or {},
                        f"connectors[{connector_index}].candidates[{candidate_index}].preview_footprint_evidence",
                    ),
                    preview_footprint_free=bool(candidate.get("preview_footprint_free", False)),
                    local_headland_candidate=bool(candidate.get("local_headland_candidate", False)),
                )
            )

        connectors.append(
            ForwardConnectorCandidateAuditResult(
                connector_id=str(item["connector_id"]),
                from_aisle_id=str(item["from_aisle_id"]),
                to_aisle_id=str(item["to_aisle_id"]),
                turn_zone_id=str(item["turn_zone_id"]),
                status=str(item.get("status", "")),
                minimum_zone_extension_m=float(item.get("minimum_zone_extension_m", 0.0)),
                local_zone_extension_limit_m=float(item.get("local_zone_extension_limit_m", 0.0)),
                candidate_count=int(item.get("candidate_count", len(candidates))),
                local_candidate_count=int(item.get("local_candidate_count", 0)),
                local_known_occupied_count=int(item.get("local_known_occupied_count", 0)),
                local_map_insufficient_count=int(item.get("local_map_insufficient_count", 0)),
                candidates=tuple(candidates),
                reason=str(item.get("reason", "")),
            )
        )

    declared = payload.get("connector_count")
    if declared is not None and int(declared) != len(connectors):
        raise AssetContractError(
            "forward_candidate_audit_count_mismatch",
            f"declared connector_count={declared}, parsed={len(connectors)}",
        )

    return ForwardConnectorCandidateAuditPlan(
        frame_id=str(payload.get("frame_id", "map")),
        platform_id=str(payload.get("platform_id", "")),
        platform_profile_sha256=str(payload.get("platform_profile_sha256", "")),
        connectors=tuple(connectors),
        source=dict(payload.get("source") or {}),
        schema=schema,
        status=str(payload.get("status", "DRAFT")),
    )
