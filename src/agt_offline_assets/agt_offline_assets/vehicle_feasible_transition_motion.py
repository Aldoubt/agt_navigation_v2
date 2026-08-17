"""A3 adapter and orchestration for locally validated headland transitions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .agricultural_coverage_ordering import ConnectorRequest
from .forward_connector import ForwardConnectorSample
from .forward_connector_candidate_audit import (
    ForwardConnectorCandidateAuditConfig,
    derive_forward_connector_candidate_audit,
)
from .forward_connector_navigation_gate import (
    ForwardConnectorNavigationGateConfig,
    GridPathEvidence,
    _evaluate_candidate,
    _preview_local_footprint,
    derive_forward_connector_navigation_gate,
)
from .navigation_grid import NavigationGridEvidence
from .reverse_fallback_admission import (
    ReverseFallbackAdmissionConfig,
    derive_reverse_fallback_admission,
)
from .reverse_primitive_connector import (
    ReversePrimitiveConnectorConfig,
    derive_reverse_primitive_connector_plan,
)
from .site_boundary import SiteBoundary
from .turn_zones import TurnZoneSet
from .vehicle_profile import CanonicalVehicleProfile
from .vehicle_feasible_motion_graph import (
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
    BOUNDED_SEARCH_NO_SOLUTION,
    EXECUTABLE,
    FORWARD_DUBINS_NAVIGATION_GATE,
    LOCAL_MOTION_EXECUTABLE,
    MAP_EVIDENCE_INSUFFICIENT,
    MIXED_EVIDENCE_REQUIRES_REVIEW,
    MotionSample,
    POLICY_REVIEW_REQUIRED,
    PROVEN_HARD_CONSTRAINT_REJECTION,
    REJECTED,
    TransitionValidation,
    UNRESOLVED,
    VehicleFeasibleMotionGraphConfig,
)
from .vehicle_feasible_service_graph import VehicleFeasibleServiceGraph


@dataclass(frozen=True)
class ConnectorBinding:
    connector_candidate_id: str
    from_service_state_id: str
    to_service_state_id: str
    from_segment_id: str
    to_segment_id: str
    side: str
    turn_zone_id: str


def _angle_error(a: float, b: float) -> float:
    return abs((float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi)


def _pose_matches(actual, expected, config: VehicleFeasibleMotionGraphConfig) -> bool:
    if len(actual) != 4 or len(expected) != 4:
        return False
    if not all(math.isfinite(float(value)) for value in (*actual, *expected)):
        return False
    position_error = math.sqrt(
        sum((float(actual[index]) - float(expected[index])) ** 2 for index in range(3))
    )
    return (
        position_error <= config.pose_position_tolerance_m
        and _angle_error(actual[3], expected[3]) <= config.pose_yaw_tolerance_rad
    )


def _unique_index(items, attr: str, label: str):
    result = {}
    for item in items:
        key = str(getattr(item, attr))
        if not key or key in result:
            raise ValueError(f"duplicate or empty {label}: {key}")
        result[key] = item
    return result


def adapt_connector_candidates(
    service_graph: VehicleFeasibleServiceGraph,
    turn_zones: TurnZoneSet,
    config: VehicleFeasibleMotionGraphConfig,
):
    """Map every A2 connector candidate one-to-one into the legacy request contract."""
    config.validate()
    if service_graph.frame_id != turn_zones.frame_id:
        raise ValueError("A3 connector adapter frame mismatch")
    states = _unique_index(service_graph.service_states, "service_state_id", "service state id")
    resources = _unique_index(service_graph.service_resources, "segment_id", "service resource id")
    zones = _unique_index(turn_zones.zones, "zone_id", "Turn Zone id")
    candidates = _unique_index(
        service_graph.connector_candidates,
        "connector_candidate_id",
        "connector candidate id",
    )

    requests: list[ConnectorRequest] = []
    bindings: dict[str, ConnectorBinding] = {}
    for candidate_id in sorted(candidates):
        candidate = candidates[candidate_id]
        source = states.get(str(candidate.from_service_state_id))
        if source is None:
            raise ValueError(f"connector {candidate_id} references missing source service state")
        target = states.get(str(candidate.to_service_state_id))
        if target is None:
            raise ValueError(f"connector {candidate_id} references missing target service state")
        if source.segment_id not in resources or target.segment_id not in resources:
            raise ValueError(f"connector {candidate_id} references missing service resource")
        if str(candidate.from_segment_id) != str(source.segment_id):
            raise ValueError(f"connector {candidate_id} source segment identity mismatch")
        if str(candidate.to_segment_id) != str(target.segment_id):
            raise ValueError(f"connector {candidate_id} target segment identity mismatch")
        zone = zones.get(str(candidate.turn_zone_id))
        if zone is None:
            raise ValueError(f"connector {candidate_id} references missing Turn Zone")
        if str(zone.side) != str(candidate.side):
            raise ValueError(f"connector {candidate_id} side does not match Turn Zone")
        supported = {str(value) for value in zone.supported_aisle_ids}
        if source.aisle_id not in supported or target.aisle_id not in supported:
            raise ValueError(f"connector {candidate_id} Turn Zone does not support both aisles")
        if not zone.allow_turn:
            raise ValueError(f"connector {candidate_id} references a Turn Zone with allow_turn=false")
        if not _pose_matches(candidate.start_pose, source.exit_pose, config):
            raise ValueError(f"connector {candidate_id} start pose does not match source exit pose")
        if not _pose_matches(candidate.goal_pose, target.entry_pose, config):
            raise ValueError(f"connector {candidate_id} goal pose does not match target entry pose")

        request = ConnectorRequest(
            connector_id=candidate_id,
            from_aisle_id=str(source.aisle_id),
            to_aisle_id=str(target.aisle_id),
            turn_zone_id=str(candidate.turn_zone_id),
            side=str(candidate.side),
            start_pose=tuple(float(value) for value in candidate.start_pose),
            goal_pose=tuple(float(value) for value in candidate.goal_pose),
        )
        requests.append(request)
        bindings[candidate_id] = ConnectorBinding(
            connector_candidate_id=candidate_id,
            from_service_state_id=str(candidate.from_service_state_id),
            to_service_state_id=str(candidate.to_service_state_id),
            from_segment_id=str(candidate.from_segment_id),
            to_segment_id=str(candidate.to_segment_id),
            side=str(candidate.side),
            turn_zone_id=str(candidate.turn_zone_id),
        )
    return tuple(requests), bindings


def _motion_samples(samples) -> tuple[MotionSample, ...]:
    output = []
    segment_index = 0
    for sample in samples:
        is_cusp = bool(getattr(sample, "is_cusp", False))
        if hasattr(sample, "segment_index"):
            segment_index = int(sample.segment_index)
        output.append(
            MotionSample(
                x=float(sample.x),
                y=float(sample.y),
                z=float(sample.z),
                yaw=float(sample.yaw),
                motion_direction=str(getattr(sample, "motion_direction", "FORWARD")),
                segment_index=segment_index,
                is_cusp=is_cusp,
            )
        )
    return tuple(output)


def _evidence_dict(evidence) -> dict[str, Any]:
    names = (
        "cell_count",
        "free_count",
        "occupied_count",
        "unknown_count",
        "free_fraction",
        "occupied_fraction",
        "unknown_fraction",
        "grid_coverage_fraction",
    )
    return {name: getattr(evidence, name) for name in names if hasattr(evidence, name)}


def _base_result(binding, candidate, *, status, proof_scope, backend, backend_status, reason,
                 path_length_m=0.0, forward_distance_m=0.0, reverse_distance_m=0.0,
                 cusp_count=0, search_expansions=0, samples=(), forward_evidence=None,
                 reverse_admission_evidence=None):
    return TransitionValidation(
        connector_candidate_id=binding.connector_candidate_id,
        from_service_state_id=binding.from_service_state_id,
        to_service_state_id=binding.to_service_state_id,
        from_segment_id=binding.from_segment_id,
        to_segment_id=binding.to_segment_id,
        side=binding.side,
        turn_zone_id=binding.turn_zone_id,
        start_pose=tuple(float(value) for value in candidate.start_pose),
        goal_pose=tuple(float(value) for value in candidate.goal_pose),
        status=status,
        proof_scope=proof_scope,
        backend=backend,
        backend_status=backend_status,
        reason=str(reason),
        path_length_m=float(path_length_m),
        forward_distance_m=float(forward_distance_m),
        reverse_distance_m=float(reverse_distance_m),
        cusp_count=int(cusp_count),
        search_expansions=int(search_expansions),
        samples=tuple(samples),
        forward_evidence=dict(forward_evidence or {}),
        reverse_admission_evidence=dict(reverse_admission_evidence or {}),
    )


def validate_transition_candidates(
    service_graph: VehicleFeasibleServiceGraph,
    turn_zones: TurnZoneSet,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleFeasibleMotionGraphConfig,
    *,
    site_boundary: SiteBoundary | None = None,
) -> tuple[TransitionValidation, ...]:
    """Validate all A2 headland candidates through the frozen R5/R6A/R6B chain."""
    config.validate()
    if not service_graph.frame_id == turn_zones.frame_id == navigation.frame_id:
        raise ValueError("A3 transition frame mismatch")
    if service_graph.platform_id != vehicle.profile_id:
        raise ValueError("A3 transition platform mismatch")
    if service_graph.platform_profile_sha256 != vehicle.profile_sha256:
        raise ValueError("A3 transition vehicle profile hash mismatch")
    if site_boundary is not None:
        site_boundary.validate(expected_frame_id=service_graph.frame_id)

    requests, bindings = adapt_connector_candidates(service_graph, turn_zones, config)
    if not requests:
        return ()
    candidates = {
        str(candidate.connector_candidate_id): candidate
        for candidate in service_graph.connector_candidates
    }
    request_by_id = {request.connector_id: request for request in requests}
    gate = derive_forward_connector_navigation_gate(
        requests,
        turn_zones,
        navigation,
        vehicle,
        ForwardConnectorNavigationGateConfig(
            preview_footprint_padding_m=config.preview_footprint_padding_m,
        ),
        site_boundary=site_boundary,
        source={"acceptance_stage": "v25_12g_a3"},
    )
    gate_by_id = {str(item.connector_id): item for item in gate.connectors}
    missing_gate = sorted(set(request_by_id) - set(gate_by_id))
    if missing_gate:
        raise ValueError("forward navigation gate dropped connector ids: " + ", ".join(missing_gate))

    results: dict[str, TransitionValidation] = {}
    unresolved_requests: list[ConnectorRequest] = []
    for connector_id in sorted(request_by_id):
        candidate = candidates[connector_id]
        binding = bindings[connector_id]
        item = gate_by_id[connector_id]
        if item.status == "PREVIEW_FOOTPRINT_FREE":
            samples = _motion_samples(item.samples)
            results[connector_id] = _base_result(
                binding,
                candidate,
                status=EXECUTABLE,
                proof_scope=LOCAL_MOTION_EXECUTABLE,
                backend=FORWARD_DUBINS_NAVIGATION_GATE,
                backend_status=item.status,
                reason=item.reason,
                path_length_m=float(item.length_m or 0.0),
                forward_distance_m=float(item.length_m or 0.0),
                samples=samples,
                forward_evidence={
                    "centerline": _evidence_dict(item.centerline_evidence),
                    "footprint": _evidence_dict(item.footprint_evidence),
                },
            )
        elif item.status == "TURN_ZONE_METADATA_INVALID":
            raise ValueError(f"adapter/gate Turn Zone metadata inconsistency for {connector_id}")
        else:
            unresolved_requests.append(request_by_id[connector_id])

    if unresolved_requests:
        audit = derive_forward_connector_candidate_audit(
            unresolved_requests,
            turn_zones,
            navigation,
            vehicle,
            ForwardConnectorCandidateAuditConfig(
                preview_footprint_padding_m=config.preview_footprint_padding_m,
            ),
            site_boundary=site_boundary,
            source={"acceptance_stage": "v25_12g_a3"},
        )
        audit_by_id = {str(item.connector_id): item for item in audit.connectors}
        missing_audit = sorted({r.connector_id for r in unresolved_requests} - set(audit_by_id))
        if missing_audit:
            raise ValueError("forward audit dropped connector ids: " + ", ".join(missing_audit))

        occupancy_ids: list[str] = []
        for request in unresolved_requests:
            connector_id = request.connector_id
            item = audit_by_id[connector_id]
            binding = bindings[connector_id]
            candidate = candidates[connector_id]
            forward_evidence = {"audit_status": item.status, "reason": item.reason}
            if item.status == "LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT":
                results[connector_id] = _base_result(
                    binding, candidate,
                    status=REJECTED,
                    proof_scope=PROVEN_HARD_CONSTRAINT_REJECTION,
                    backend=FORWARD_DUBINS_NAVIGATION_GATE,
                    backend_status=item.status,
                    reason=item.reason,
                    forward_evidence=forward_evidence,
                )
            elif item.status == "LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT":
                results[connector_id] = _base_result(
                    binding, candidate,
                    status=UNRESOLVED,
                    proof_scope=MAP_EVIDENCE_INSUFFICIENT,
                    backend=FORWARD_DUBINS_NAVIGATION_GATE,
                    backend_status=item.status,
                    reason=item.reason,
                    forward_evidence=forward_evidence,
                )
            elif item.status == "LOCAL_FORWARD_MIXED_EVIDENCE":
                results[connector_id] = _base_result(
                    binding, candidate,
                    status=UNRESOLVED,
                    proof_scope=MIXED_EVIDENCE_REQUIRES_REVIEW,
                    backend=FORWARD_DUBINS_NAVIGATION_GATE,
                    backend_status=item.status,
                    reason=item.reason,
                    forward_evidence=forward_evidence,
                )
            elif item.status == "NO_FORWARD_DUBINS_CANDIDATE":
                results[connector_id] = _base_result(
                    binding, candidate,
                    status=REJECTED,
                    proof_scope=PROVEN_HARD_CONSTRAINT_REJECTION,
                    backend=FORWARD_DUBINS_NAVIGATION_GATE,
                    backend_status=item.status,
                    reason=item.reason,
                    forward_evidence=forward_evidence,
                )
            elif item.status == "LOCAL_FORWARD_OCCUPANCY_BLOCKED":
                occupancy_ids.append(connector_id)
            elif item.status == "FORWARD_PREVIEW_FREE":
                raise ValueError(f"forward gate/audit disagreement for {connector_id}")
            else:
                results[connector_id] = _base_result(
                    binding, candidate,
                    status=UNRESOLVED,
                    proof_scope=POLICY_REVIEW_REQUIRED,
                    backend=FORWARD_DUBINS_NAVIGATION_GATE,
                    backend_status=item.status,
                    reason=item.reason,
                    forward_evidence=forward_evidence,
                )

        if occupancy_ids:
            admission = derive_reverse_fallback_admission(
                audit,
                ReverseFallbackAdmissionConfig(operator_approved_mixed_connector_ids=()),
                source={"acceptance_stage": "v25_12g_a3"},
            )
            admission_by_id = {str(item.connector_id): item for item in admission.items}
            admitted = set(admission.eligible_connector_ids)
            expected_admitted = set(occupancy_ids)
            if not expected_admitted.issubset(admitted):
                missing = sorted(expected_admitted - admitted)
                raise ValueError("R6A failed to admit occupancy-blocked connector ids: " + ", ".join(missing))
            reverse_requests = tuple(request_by_id[item] for item in sorted(expected_admitted))
            reverse_plan = derive_reverse_primitive_connector_plan(
                reverse_requests,
                admission,
                turn_zones,
                navigation,
                vehicle,
                ReversePrimitiveConnectorConfig(
                    preview_footprint_padding_m=config.preview_footprint_padding_m,
                ),
                site_boundary=site_boundary,
                source={"acceptance_stage": "v25_12g_a3"},
            )
            reverse_by_id = {str(item.connector_id): item for item in reverse_plan.connectors}
            missing_reverse = sorted(expected_admitted - set(reverse_by_id))
            if missing_reverse:
                raise ValueError("R6B dropped admitted connector ids: " + ", ".join(missing_reverse))

            local_footprint = _preview_local_footprint(
                vehicle,
                config.preview_footprint_padding_m,
            )
            for connector_id in sorted(expected_admitted):
                candidate = candidates[connector_id]
                binding = bindings[connector_id]
                reverse = reverse_by_id[connector_id]
                admission_item = admission_by_id.get(connector_id)
                admission_evidence = {
                    "decision": getattr(admission_item, "decision", "ELIGIBLE_REVERSE_FALLBACK"),
                    "reason": getattr(admission_item, "reason", ""),
                }
                forward_evidence = {
                    "audit_status": audit_by_id[connector_id].status,
                    "reason": audit_by_id[connector_id].reason,
                }
                if reverse.status in {
                    "REVERSE_PRIMITIVE_PREVIEW_FREE",
                    "FORWARD_PRIMITIVE_PREVIEW_FREE",
                }:
                    results[connector_id] = _base_result(
                        binding, candidate,
                        status=EXECUTABLE,
                        proof_scope=LOCAL_MOTION_EXECUTABLE,
                        backend=BOUNDED_REVERSE_PRIMITIVE_SEARCH,
                        backend_status=reverse.status,
                        reason=reverse.reason,
                        path_length_m=float(reverse.path_length_m or 0.0),
                        forward_distance_m=float(reverse.forward_distance_m),
                        reverse_distance_m=float(reverse.reverse_distance_m),
                        cusp_count=int(reverse.cusp_count),
                        search_expansions=int(reverse.search_expansions),
                        samples=_motion_samples(reverse.samples),
                        forward_evidence=forward_evidence,
                        reverse_admission_evidence=admission_evidence,
                    )
                elif reverse.status == "SITE_BOUNDARY_CONFLICT":
                    results[connector_id] = _base_result(
                        binding, candidate,
                        status=REJECTED,
                        proof_scope=PROVEN_HARD_CONSTRAINT_REJECTION,
                        backend=BOUNDED_REVERSE_PRIMITIVE_SEARCH,
                        backend_status=reverse.status,
                        reason=reverse.reason,
                        search_expansions=int(reverse.search_expansions),
                        forward_evidence=forward_evidence,
                        reverse_admission_evidence=admission_evidence,
                    )
                elif reverse.status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION":
                    reason = str(reverse.reason).replace("global infeasibility", "infeasibility")
                    results[connector_id] = _base_result(
                        binding, candidate,
                        status=REJECTED,
                        proof_scope=BOUNDED_SEARCH_NO_SOLUTION,
                        backend=BOUNDED_REVERSE_PRIMITIVE_SEARCH,
                        backend_status=reverse.status,
                        reason=reason,
                        search_expansions=int(reverse.search_expansions),
                        forward_evidence=forward_evidence,
                        reverse_admission_evidence=admission_evidence,
                    )
                elif reverse.status == "R6B_START_FOOTPRINT_NOT_FREE":
                    request = request_by_id[connector_id]
                    probe = (
                        ForwardConnectorSample(
                            x=float(request.start_pose[0]),
                            y=float(request.start_pose[1]),
                            z=float(request.start_pose[2]),
                            yaw=float(request.start_pose[3]),
                        ),
                    )
                    _center, footprint = _evaluate_candidate(probe, navigation, local_footprint)
                    if footprint.occupied_fraction > 0.0:
                        status = REJECTED
                        proof_scope = PROVEN_HARD_CONSTRAINT_REJECTION
                    else:
                        status = UNRESOLVED
                        proof_scope = MAP_EVIDENCE_INSUFFICIENT
                    results[connector_id] = _base_result(
                        binding, candidate,
                        status=status,
                        proof_scope=proof_scope,
                        backend=BOUNDED_REVERSE_PRIMITIVE_SEARCH,
                        backend_status=reverse.status,
                        reason=reverse.reason,
                        search_expansions=int(reverse.search_expansions),
                        forward_evidence=forward_evidence,
                        reverse_admission_evidence=admission_evidence,
                    )
                else:
                    results[connector_id] = _base_result(
                        binding, candidate,
                        status=UNRESOLVED,
                        proof_scope=POLICY_REVIEW_REQUIRED,
                        backend=BOUNDED_REVERSE_PRIMITIVE_SEARCH,
                        backend_status=str(reverse.status),
                        reason=str(reverse.reason),
                        search_expansions=int(reverse.search_expansions),
                        forward_evidence=forward_evidence,
                        reverse_admission_evidence=admission_evidence,
                    )

    missing_results = sorted(set(request_by_id) - set(results))
    if missing_results:
        raise ValueError("A3 transition validation dropped connector ids: " + ", ".join(missing_results))
    return tuple(results[connector_id] for connector_id in sorted(results))
