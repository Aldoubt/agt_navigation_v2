from pathlib import Path

import pytest

from agt_offline_assets.forward_connector_candidate_audit import (
    ForwardConnectorCandidateAuditPlan,
    ForwardConnectorCandidateAuditResult,
    write_forward_connector_candidate_audit,
)
from agt_offline_assets.reverse_fallback_admission import (
    ReverseFallbackAdmissionConfig,
    derive_reverse_fallback_admission,
    reverse_fallback_admission_to_dict,
)
from agt_offline_assets.route_diagnostic_io import load_forward_connector_candidate_audit


def _result(connector_id: str, status: str) -> ForwardConnectorCandidateAuditResult:
    index = int(connector_id.split("_")[-1])
    return ForwardConnectorCandidateAuditResult(
        connector_id=connector_id,
        from_aisle_id=f"aisle_{index:03d}",
        to_aisle_id=f"aisle_{index + 1:03d}",
        turn_zone_id="turn_high_u" if index % 2 else "turn_low_u",
        status=status,
        minimum_zone_extension_m=0.2,
        local_zone_extension_limit_m=0.45,
        candidate_count=2,
        local_candidate_count=1,
        local_known_occupied_count=1 if "OCCUPANCY_BLOCKED" in status else 0,
        local_map_insufficient_count=1 if "MAP_EVIDENCE_INSUFFICIENT" in status else 0,
        candidates=(),
        reason="fixture",
    )


def _audit() -> ForwardConnectorCandidateAuditPlan:
    return ForwardConnectorCandidateAuditPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="deadbeef",
        connectors=(
            _result("connector_001", "LOCAL_FORWARD_OCCUPANCY_BLOCKED"),
            _result("connector_002", "LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT"),
            _result("connector_003", "LOCAL_FORWARD_MIXED_EVIDENCE"),
            _result("connector_004", "FORWARD_PREVIEW_FREE"),
        ),
        source={"fixture": "r5_6"},
    )


def test_admission_auto_selects_only_known_occupancy_blocked():
    plan = derive_reverse_fallback_admission(_audit())
    assert plan.eligible_count == 1
    assert plan.hold_map_review_count == 1
    assert plan.hold_mixed_evidence_count == 1
    assert plan.keep_forward_count == 1
    assert plan.eligible_connector_ids == ("connector_001",)
    decisions = {item.connector_id: item.decision for item in plan.items}
    assert decisions["connector_001"] == "ELIGIBLE_REVERSE_FALLBACK"
    assert decisions["connector_002"] == "HOLD_MAP_REVIEW"
    assert decisions["connector_003"] == "HOLD_MIXED_EVIDENCE"
    assert decisions["connector_004"] == "KEEP_FORWARD"


def test_mixed_evidence_requires_explicit_operator_approval():
    plan = derive_reverse_fallback_admission(
        _audit(),
        ReverseFallbackAdmissionConfig(
            operator_approved_mixed_connector_ids=("connector_003",)
        ),
    )
    assert plan.eligible_count == 2
    assert "connector_003" in plan.eligible_connector_ids
    assert plan.hold_mixed_evidence_count == 0


def test_unknown_operator_approval_fails_closed():
    with pytest.raises(ValueError):
        derive_reverse_fallback_admission(
            _audit(),
            ReverseFallbackAdmissionConfig(
                operator_approved_mixed_connector_ids=("connector_999",)
            ),
        )


def test_candidate_audit_round_trip_can_feed_admission(tmp_path: Path):
    path = tmp_path / "forward_connector_candidate_audit.yaml"
    write_forward_connector_candidate_audit(_audit(), path)
    loaded = load_forward_connector_candidate_audit(path)
    assert len(loaded.connectors) == 4
    assert loaded.connectors[0].status == "LOCAL_FORWARD_OCCUPANCY_BLOCKED"
    admission = derive_reverse_fallback_admission(loaded)
    assert admission.eligible_connector_ids == ("connector_001",)


def test_serialization_preserves_r6b_input_selection_boundary():
    payload = reverse_fallback_admission_to_dict(
        derive_reverse_fallback_admission(_audit())
    )
    assert payload["schema"] == "agt_reverse_fallback_admission/v1"
    assert payload["status"] == "DRAFT"
    assert payload["eligible_connector_ids"] == ["connector_001"]
    assert payload["source"]["admission_policy"] == "AUTO_ONLY_LOCAL_FORWARD_OCCUPANCY_BLOCKED"
    assert payload["source"]["map_insufficient_never_auto_admitted"] is True
    assert payload["source"]["validation_scope"] == "R6B_INPUT_SELECTION_NOT_ROUTE_READY"
