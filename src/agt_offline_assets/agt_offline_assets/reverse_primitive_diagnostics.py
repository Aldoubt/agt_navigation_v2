"""Diagnostic-only accounting and classification for the bounded R6B backend.

This module deliberately contains no planner control flow.  It only validates,
serializes and classifies observations produced by the existing bounded search.
The classification vocabulary is descriptive evidence, not a proof of global
infeasibility and not route-readiness evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


R6B_DIAGNOSTIC_SCHEMA = "agt_r6b_connector_diagnostics/v1"

SEARCH_ENVELOPE_LIMITED = "SEARCH_ENVELOPE_LIMITED"
SITE_BOUNDARY_LIMITED = "SITE_BOUNDARY_LIMITED"
FOOTPRINT_GRID_LIMITED = "FOOTPRINT_GRID_LIMITED"
STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED = (
    "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED"
)
GOAL_CONNECTION_LIMITED = "GOAL_CONNECTION_LIMITED"
MOTION_PRIMITIVE_LIMITED = "MOTION_PRIMITIVE_LIMITED"
PATH_OR_CUSP_ENVELOPE_LIMITED = "PATH_OR_CUSP_ENVELOPE_LIMITED"
MIXED_LIMITATION = "MIXED_LIMITATION"
INCONCLUSIVE = "INCONCLUSIVE"

FAILURE_CLASSES = frozenset(
    {
        SEARCH_ENVELOPE_LIMITED,
        SITE_BOUNDARY_LIMITED,
        FOOTPRINT_GRID_LIMITED,
        STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED,
        GOAL_CONNECTION_LIMITED,
        MOTION_PRIMITIVE_LIMITED,
        PATH_OR_CUSP_ENVELOPE_LIMITED,
        MIXED_LIMITATION,
        INCONCLUSIVE,
    }
)

DOMINANT_REJECTION_FRACTION = 0.50
MIXED_REJECTION_FRACTION = 0.25
DOMINANCE_REJECTION_FRACTION = 0.60
DOMINANCE_ENQUEUED_FRACTION_MAX = 0.15
CUSP_SATURATION_FRACTION = 0.50
LITTLE_PROGRESS_FRACTION = 0.20


@dataclass(frozen=True)
class ReversePrimitiveSearchDiagnostics:
    failure_class: str | None = None
    search_started: bool = False
    search_expansions: int = 0
    queue_exhausted: bool = False
    expansion_budget_reached: bool = False

    start_goal_position_error_m: float | None = None
    best_goal_position_error_m: float | None = None
    best_goal_yaw_error_rad: float | None = None
    best_goal_distance_state_direction: str | None = None
    best_goal_distance_cusp_count: int | None = None

    nodes_popped: int = 0
    stale_nodes_skipped: int = 0

    cusp_switches_considered: int = 0
    cusp_switches_rejected_state_dominance: int = 0
    cusp_switches_enqueued: int = 0
    nodes_at_max_cusps: int = 0

    primitive_edges_considered: int = 0
    primitive_edges_rejected_search_envelope: int = 0
    primitive_edges_rejected_site_boundary: int = 0
    primitive_edges_rejected_navigation_grid: int = 0
    primitive_edges_rejected_path_length: int = 0
    primitive_edges_rejected_state_dominance: int = 0
    primitive_edges_enqueued: int = 0

    goal_tolerance_checks: int = 0
    goal_tolerance_successes: int = 0

    goal_shot_attempts: int = 0
    goal_shot_reverse_direction_blocked: int = 0
    goal_shot_candidates_considered: int = 0
    goal_shot_rejected_path_length: int = 0
    goal_shot_rejected_search_envelope: int = 0
    goal_shot_rejected_site_boundary: int = 0
    goal_shot_rejected_navigation_grid: int = 0
    goal_shot_successes: int = 0

    schema: str = R6B_DIAGNOSTIC_SCHEMA


_COUNTER_FIELDS = (
    "search_expansions",
    "nodes_popped",
    "stale_nodes_skipped",
    "cusp_switches_considered",
    "cusp_switches_rejected_state_dominance",
    "cusp_switches_enqueued",
    "nodes_at_max_cusps",
    "primitive_edges_considered",
    "primitive_edges_rejected_search_envelope",
    "primitive_edges_rejected_site_boundary",
    "primitive_edges_rejected_navigation_grid",
    "primitive_edges_rejected_path_length",
    "primitive_edges_rejected_state_dominance",
    "primitive_edges_enqueued",
    "goal_tolerance_checks",
    "goal_tolerance_successes",
    "goal_shot_attempts",
    "goal_shot_reverse_direction_blocked",
    "goal_shot_candidates_considered",
    "goal_shot_rejected_path_length",
    "goal_shot_rejected_search_envelope",
    "goal_shot_rejected_site_boundary",
    "goal_shot_rejected_navigation_grid",
    "goal_shot_successes",
)

_OPTIONAL_ERROR_FIELDS = (
    "start_goal_position_error_m",
    "best_goal_position_error_m",
    "best_goal_yaw_error_rad",
)


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be an integer >= 0")
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be an integer >= 0") from exc
    if result < 0 or result != value:
        raise ValueError(f"{label} must be an integer >= 0")
    return result


def _optional_nonnegative_float(value: Any, label: str) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{label} must be finite and >= 0 when present")
    return result


def _bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be boolean")
    return value


def validate_reverse_primitive_search_diagnostics(
    diagnostics: ReversePrimitiveSearchDiagnostics,
) -> None:
    if diagnostics.schema != R6B_DIAGNOSTIC_SCHEMA:
        raise ValueError(
            f"diagnostic schema must be {R6B_DIAGNOSTIC_SCHEMA}, got {diagnostics.schema}"
        )
    if (
        diagnostics.failure_class is not None
        and diagnostics.failure_class not in FAILURE_CLASSES
    ):
        raise ValueError(f"invalid R6B diagnostic failure class: {diagnostics.failure_class}")
    if diagnostics.best_goal_distance_state_direction not in {None, "FORWARD", "REVERSE"}:
        raise ValueError(
            "best_goal_distance_state_direction must be FORWARD, REVERSE or null"
        )

    for name in _COUNTER_FIELDS:
        _nonnegative_int(getattr(diagnostics, name), name)
    for name in _OPTIONAL_ERROR_FIELDS:
        _optional_nonnegative_float(getattr(diagnostics, name), name)
    if diagnostics.best_goal_distance_cusp_count is not None:
        _nonnegative_int(
            diagnostics.best_goal_distance_cusp_count,
            "best_goal_distance_cusp_count",
        )

    if diagnostics.queue_exhausted and diagnostics.expansion_budget_reached:
        raise ValueError("queue exhaustion and expansion budget termination are exclusive")
    if not diagnostics.search_started and (
        diagnostics.queue_exhausted or diagnostics.expansion_budget_reached
    ):
        raise ValueError("search termination flags require search_started=true")

    if diagnostics.nodes_popped != (
        diagnostics.search_expansions + diagnostics.stale_nodes_skipped
    ):
        raise ValueError("R6B accounting identity mismatch: nodes_popped")
    if diagnostics.cusp_switches_considered != (
        diagnostics.cusp_switches_rejected_state_dominance
        + diagnostics.cusp_switches_enqueued
    ):
        raise ValueError("R6B accounting identity mismatch: cusp switches")
    if diagnostics.primitive_edges_considered != (
        diagnostics.primitive_edges_rejected_search_envelope
        + diagnostics.primitive_edges_rejected_site_boundary
        + diagnostics.primitive_edges_rejected_navigation_grid
        + diagnostics.primitive_edges_rejected_path_length
        + diagnostics.primitive_edges_rejected_state_dominance
        + diagnostics.primitive_edges_enqueued
    ):
        raise ValueError("R6B accounting identity mismatch: primitive edges")
    if diagnostics.goal_shot_candidates_considered != (
        diagnostics.goal_shot_rejected_path_length
        + diagnostics.goal_shot_rejected_search_envelope
        + diagnostics.goal_shot_rejected_site_boundary
        + diagnostics.goal_shot_rejected_navigation_grid
        + diagnostics.goal_shot_successes
    ):
        raise ValueError("R6B accounting identity mismatch: goal-shot candidates")
    if diagnostics.goal_tolerance_successes > diagnostics.goal_tolerance_checks:
        raise ValueError("goal tolerance successes exceed checks")
    if diagnostics.goal_shot_successes > diagnostics.goal_shot_attempts:
        raise ValueError("goal-shot successes exceed attempts")


def reverse_primitive_search_diagnostics_to_dict(
    diagnostics: ReversePrimitiveSearchDiagnostics,
) -> dict[str, Any]:
    validate_reverse_primitive_search_diagnostics(diagnostics)
    return {
        "schema": diagnostics.schema,
        "failure_class": diagnostics.failure_class,
        "search_started": diagnostics.search_started,
        "search_expansions": diagnostics.search_expansions,
        "queue_exhausted": diagnostics.queue_exhausted,
        "expansion_budget_reached": diagnostics.expansion_budget_reached,
        "start_goal_position_error_m": diagnostics.start_goal_position_error_m,
        "best_goal_position_error_m": diagnostics.best_goal_position_error_m,
        "best_goal_yaw_error_rad": diagnostics.best_goal_yaw_error_rad,
        "best_goal_distance_state_direction": diagnostics.best_goal_distance_state_direction,
        "best_goal_distance_cusp_count": diagnostics.best_goal_distance_cusp_count,
        "nodes_popped": diagnostics.nodes_popped,
        "stale_nodes_skipped": diagnostics.stale_nodes_skipped,
        "cusp_switches_considered": diagnostics.cusp_switches_considered,
        "cusp_switches_rejected_state_dominance": diagnostics.cusp_switches_rejected_state_dominance,
        "cusp_switches_enqueued": diagnostics.cusp_switches_enqueued,
        "nodes_at_max_cusps": diagnostics.nodes_at_max_cusps,
        "primitive_edges_considered": diagnostics.primitive_edges_considered,
        "primitive_edges_rejected_search_envelope": diagnostics.primitive_edges_rejected_search_envelope,
        "primitive_edges_rejected_site_boundary": diagnostics.primitive_edges_rejected_site_boundary,
        "primitive_edges_rejected_navigation_grid": diagnostics.primitive_edges_rejected_navigation_grid,
        "primitive_edges_rejected_path_length": diagnostics.primitive_edges_rejected_path_length,
        "primitive_edges_rejected_state_dominance": diagnostics.primitive_edges_rejected_state_dominance,
        "primitive_edges_enqueued": diagnostics.primitive_edges_enqueued,
        "goal_tolerance_checks": diagnostics.goal_tolerance_checks,
        "goal_tolerance_successes": diagnostics.goal_tolerance_successes,
        "goal_shot_attempts": diagnostics.goal_shot_attempts,
        "goal_shot_reverse_direction_blocked": diagnostics.goal_shot_reverse_direction_blocked,
        "goal_shot_candidates_considered": diagnostics.goal_shot_candidates_considered,
        "goal_shot_rejected_path_length": diagnostics.goal_shot_rejected_path_length,
        "goal_shot_rejected_search_envelope": diagnostics.goal_shot_rejected_search_envelope,
        "goal_shot_rejected_site_boundary": diagnostics.goal_shot_rejected_site_boundary,
        "goal_shot_rejected_navigation_grid": diagnostics.goal_shot_rejected_navigation_grid,
        "goal_shot_successes": diagnostics.goal_shot_successes,
    }


def reverse_primitive_search_diagnostics_from_dict(
    raw: Mapping[str, Any],
) -> ReversePrimitiveSearchDiagnostics:
    if not isinstance(raw, Mapping):
        raise ValueError("R6B diagnostics must be a mapping")
    schema = str(raw.get("schema", ""))
    if schema != R6B_DIAGNOSTIC_SCHEMA:
        raise ValueError(
            f"diagnostic schema must be {R6B_DIAGNOSTIC_SCHEMA}, got {schema}"
        )
    failure_class = raw.get("failure_class")
    if failure_class is not None:
        failure_class = str(failure_class)
        if failure_class not in FAILURE_CLASSES:
            raise ValueError(f"invalid R6B diagnostic failure class: {failure_class}")

    kwargs: dict[str, Any] = {
        "schema": schema,
        "failure_class": failure_class,
        "search_started": _bool(raw.get("search_started", False), "search_started"),
        "queue_exhausted": _bool(raw.get("queue_exhausted", False), "queue_exhausted"),
        "expansion_budget_reached": _bool(
            raw.get("expansion_budget_reached", False),
            "expansion_budget_reached",
        ),
    }
    for name in _COUNTER_FIELDS:
        kwargs[name] = _nonnegative_int(raw.get(name, 0), name)
    for name in _OPTIONAL_ERROR_FIELDS:
        kwargs[name] = _optional_nonnegative_float(raw.get(name), name)

    direction = raw.get("best_goal_distance_state_direction")
    if direction is not None:
        direction = str(direction)
        if direction not in {"FORWARD", "REVERSE"}:
            raise ValueError(
                "best_goal_distance_state_direction must be FORWARD, REVERSE or null"
            )
    kwargs["best_goal_distance_state_direction"] = direction

    cusp_count = raw.get("best_goal_distance_cusp_count")
    kwargs["best_goal_distance_cusp_count"] = (
        None
        if cusp_count is None
        else _nonnegative_int(cusp_count, "best_goal_distance_cusp_count")
    )

    diagnostics = ReversePrimitiveSearchDiagnostics(**kwargs)
    validate_reverse_primitive_search_diagnostics(diagnostics)
    return diagnostics


def _fraction(numerator: int, denominator: int) -> float:
    return float(numerator) / float(max(int(denominator), 1))


def classify_reverse_primitive_failure(
    diagnostics: ReversePrimitiveSearchDiagnostics,
    *,
    max_cusps: int,
) -> str:
    """Classify a bounded no-solution diagnostic record using frozen A3.5 rules."""
    if max_cusps < 0:
        raise ValueError("max_cusps must be >= 0")

    geometry_candidate_count = (
        diagnostics.primitive_edges_considered
        + diagnostics.goal_shot_candidates_considered
    )
    combined = {
        SEARCH_ENVELOPE_LIMITED: (
            diagnostics.primitive_edges_rejected_search_envelope
            + diagnostics.goal_shot_rejected_search_envelope
        ),
        SITE_BOUNDARY_LIMITED: (
            diagnostics.primitive_edges_rejected_site_boundary
            + diagnostics.goal_shot_rejected_site_boundary
        ),
        FOOTPRINT_GRID_LIMITED: (
            diagnostics.primitive_edges_rejected_navigation_grid
            + diagnostics.goal_shot_rejected_navigation_grid
        ),
        PATH_OR_CUSP_ENVELOPE_LIMITED: (
            diagnostics.primitive_edges_rejected_path_length
            + diagnostics.goal_shot_rejected_path_length
        ),
    }
    fractions = {
        name: _fraction(count, geometry_candidate_count)
        for name, count in combined.items()
    }

    dominant = {
        name for name, value in fractions.items()
        if value >= DOMINANT_REJECTION_FRACTION
    }
    if len(dominant) >= 2:
        return MIXED_LIMITATION
    if len(dominant) == 1:
        return next(iter(dominant))

    primitive_count = diagnostics.primitive_edges_considered
    dominance_rejection_fraction = _fraction(
        diagnostics.primitive_edges_rejected_state_dominance,
        primitive_count,
    )
    primitive_enqueued_fraction = _fraction(
        diagnostics.primitive_edges_enqueued,
        primitive_count,
    )
    state_dominance_signal = (
        primitive_count > 0
        and dominance_rejection_fraction >= DOMINANCE_REJECTION_FRACTION
        and primitive_enqueued_fraction <= DOMINANCE_ENQUEUED_FRACTION_MAX
    )
    cusp_saturation_signal = (
        diagnostics.search_expansions > 0
        and _fraction(
            diagnostics.nodes_at_max_cusps,
            diagnostics.search_expansions,
        )
        >= CUSP_SATURATION_FRACTION
        and diagnostics.best_goal_distance_cusp_count == max_cusps
    )
    goal_connection_signal = (
        diagnostics.best_goal_position_error_m is not None
        and diagnostics.goal_tolerance_successes == 0
        and diagnostics.goal_shot_successes == 0
        and (
            diagnostics.goal_shot_attempts > 0
            or diagnostics.goal_shot_reverse_direction_blocked > 0
        )
    )

    moderate_or_structural: set[str] = {
        name for name, value in fractions.items()
        if value >= MIXED_REJECTION_FRACTION
    }
    structural: set[str] = set()
    if state_dominance_signal:
        structural.add(STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED)
    if cusp_saturation_signal:
        structural.add(PATH_OR_CUSP_ENVELOPE_LIMITED)
    if goal_connection_signal:
        structural.add(GOAL_CONNECTION_LIMITED)
    moderate_or_structural.update(structural)

    if len(moderate_or_structural) >= 2:
        return MIXED_LIMITATION
    if len(structural) == 1 and len(moderate_or_structural) == 1:
        return next(iter(structural))

    little_progress = False
    if (
        diagnostics.start_goal_position_error_m is not None
        and diagnostics.best_goal_position_error_m is not None
    ):
        progress = (
            diagnostics.start_goal_position_error_m
            - diagnostics.best_goal_position_error_m
        ) / max(diagnostics.start_goal_position_error_m, 1.0e-12)
        little_progress = progress <= LITTLE_PROGRESS_FRACTION

    geometric_values = tuple(fractions.values())
    motion_primitive_signal = (
        diagnostics.queue_exhausted
        and primitive_count > 0
        and all(value < MIXED_REJECTION_FRACTION for value in geometric_values)
        and dominance_rejection_fraction < MIXED_REJECTION_FRACTION
        and primitive_enqueued_fraction >= MIXED_REJECTION_FRACTION
        and little_progress
        and not goal_connection_signal
        and not cusp_saturation_signal
    )
    if motion_primitive_signal:
        return MOTION_PRIMITIVE_LIMITED
    return INCONCLUSIVE
