"""R6A admission gate for reverse-aware agricultural connectors.

The forward candidate audit is the evidence boundary for entering reverse
planning.  This module deliberately does not generate reverse motion.  It only
classifies which connector requests may be handed to R6B.

Automatic admission is conservative:

- ``LOCAL_FORWARD_OCCUPANCY_BLOCKED`` -> eligible for reverse fallback
- ``LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT`` -> hold for map review
- ``LOCAL_FORWARD_MIXED_EVIDENCE`` -> hold unless explicitly operator-approved
- ``FORWARD_PREVIEW_FREE`` -> keep the forward solution path

This prevents a reverse planner from hiding incomplete map evidence.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml

from .forward_connector_candidate_audit import ForwardConnectorCandidateAuditPlan


REVERSE_FALLBACK_ADMISSION_SCHEMA = "agt_reverse_fallback_admission/v1"


@dataclass(frozen=True)
class ReverseFallbackAdmissionConfig:
    operator_approved_mixed_connector_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class ReverseFallbackAdmissionItem:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    forward_audit_status: str
    decision: str
    reason: str


@dataclass(frozen=True)
class ReverseFallbackAdmissionPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    items: tuple[ReverseFallbackAdmissionItem, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = REVERSE_FALLBACK_ADMISSION_SCHEMA
    status: str = "DRAFT"

    @property
    def eligible_count(self) -> int:
        return sum(item.decision == "ELIGIBLE_REVERSE_FALLBACK" for item in self.items)

    @property
    def hold_map_review_count(self) -> int:
        return sum(item.decision == "HOLD_MAP_REVIEW" for item in self.items)

    @property
    def hold_mixed_evidence_count(self) -> int:
        return sum(item.decision == "HOLD_MIXED_EVIDENCE" for item in self.items)

    @property
    def keep_forward_count(self) -> int:
        return sum(item.decision == "KEEP_FORWARD" for item in self.items)

    @property
    def eligible_connector_ids(self) -> tuple[str, ...]:
        return tuple(
            item.connector_id
            for item in self.items
            if item.decision == "ELIGIBLE_REVERSE_FALLBACK"
        )


def derive_reverse_fallback_admission(
    audit: ForwardConnectorCandidateAuditPlan,
    config: ReverseFallbackAdmissionConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> ReverseFallbackAdmissionPlan:
    """Translate R5.6 evidence classes into an explicit R6B admission asset."""
    cfg = config or ReverseFallbackAdmissionConfig()
    approved_mixed = set(cfg.operator_approved_mixed_connector_ids)
    items: list[ReverseFallbackAdmissionItem] = []

    known_ids = {item.connector_id for item in audit.connectors}
    unknown_approvals = sorted(approved_mixed - known_ids)
    if unknown_approvals:
        raise ValueError(
            "operator-approved mixed connector ids are not present in audit: "
            + ", ".join(unknown_approvals)
        )

    for item in audit.connectors:
        status = item.status
        if status == "LOCAL_FORWARD_OCCUPANCY_BLOCKED":
            decision = "ELIGIBLE_REVERSE_FALLBACK"
            reason = (
                "all locally relevant forward Dubins candidates are sufficiently mapped "
                "and intersect OCCUPIED cells"
            )
        elif status == "LOCAL_FORWARD_MIXED_EVIDENCE":
            if item.connector_id in approved_mixed:
                decision = "ELIGIBLE_REVERSE_FALLBACK"
                reason = (
                    "mixed forward evidence was explicitly operator-approved for reverse fallback; "
                    "approval does not promote the final route to READY"
                )
            else:
                decision = "HOLD_MIXED_EVIDENCE"
                reason = (
                    "forward audit contains both known OCCUPIED conflicts and insufficient-map "
                    "alternatives; operator review is required before reverse planning"
                )
        elif status == "LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT":
            decision = "HOLD_MAP_REVIEW"
            reason = (
                "local forward alternatives require UNKNOWN/incomplete map evidence; "
                "reverse planning must not hide the map evidence gap"
            )
        elif status == "FORWARD_PREVIEW_FREE":
            decision = "KEEP_FORWARD"
            reason = "at least one forward candidate is preview-footprint free"
        else:
            decision = "HOLD_POLICY_REVIEW"
            reason = f"forward audit status {status} requires explicit policy review"

        items.append(
            ReverseFallbackAdmissionItem(
                connector_id=item.connector_id,
                from_aisle_id=item.from_aisle_id,
                to_aisle_id=item.to_aisle_id,
                turn_zone_id=item.turn_zone_id,
                forward_audit_status=status,
                decision=decision,
                reason=reason,
            )
        )

    merged_source = dict(audit.source)
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "admission_policy": "AUTO_ONLY_LOCAL_FORWARD_OCCUPANCY_BLOCKED",
            "mixed_evidence_requires_operator_approval": True,
            "map_insufficient_never_auto_admitted": True,
            "validation_scope": "R6B_INPUT_SELECTION_NOT_ROUTE_READY",
        }
    )
    return ReverseFallbackAdmissionPlan(
        frame_id=audit.frame_id,
        platform_id=audit.platform_id,
        platform_profile_sha256=audit.platform_profile_sha256,
        items=tuple(items),
        source=merged_source,
    )


def reverse_fallback_admission_to_dict(plan: ReverseFallbackAdmissionPlan) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source": dict(plan.source),
        "connector_count": len(plan.items),
        "summary": {
            "eligible_reverse_fallback": plan.eligible_count,
            "hold_map_review": plan.hold_map_review_count,
            "hold_mixed_evidence": plan.hold_mixed_evidence_count,
            "keep_forward": plan.keep_forward_count,
        },
        "eligible_connector_ids": list(plan.eligible_connector_ids),
        "connectors": [
            {
                "connector_id": item.connector_id,
                "from_aisle_id": item.from_aisle_id,
                "to_aisle_id": item.to_aisle_id,
                "turn_zone_id": item.turn_zone_id,
                "forward_audit_status": item.forward_audit_status,
                "decision": item.decision,
                "reason": item.reason,
            }
            for item in plan.items
        ],
    }


def write_reverse_fallback_admission(
    plan: ReverseFallbackAdmissionPlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            reverse_fallback_admission_to_dict(plan),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
