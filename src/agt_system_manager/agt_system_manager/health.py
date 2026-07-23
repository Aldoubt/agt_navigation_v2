"""Pure-Python health evaluation.

The evaluator deliberately consumes observations supplied by an adapter. It does
not invoke the ``ros2`` CLI and has no ROS dependency, which keeps the threshold
and recovery behavior testable with deterministic timestamps.
"""

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Mapping


UNKNOWN = "UNKNOWN"
OK = "OK"
WARN = "WARN"
ERROR = "ERROR"
_RANK = {UNKNOWN: 0, OK: 1, WARN: 2, ERROR: 3}


@dataclass(frozen=True)
class TopicObservation:
    """Receipt statistics for one topic.

    ``last_seen`` and ``first_seen`` use the evaluator clock. A ROS adapter may
    use ROS time or monotonic time, but it must use the same clock for ``now``.
    """

    count: int = 0
    first_seen: float | None = None
    last_seen: float | None = None
    message_type: str = ""

    @property
    def age(self) -> float:
        return float("inf") if self.last_seen is None else 0.0

    def rate(self, now: float) -> float:
        del now
        if self.count < 2 or self.first_seen is None or self.last_seen is None:
            return 0.0
        span = self.last_seen - self.first_seen
        return (self.count - 1) / span if span > 0.0 else 0.0

    def age_at(self, now: float) -> float:
        if self.last_seen is None:
            return float("inf")
        return max(0.0, now - self.last_seen)


@dataclass
class HealthComponent:
    component_id: str
    display_name: str
    state: str = UNKNOWN
    required: bool = False
    present: bool = False
    observed_rate_hz: float = 0.0
    message_age_sec: float = float("inf")
    message_count: int = 0
    missing_topics: list[str] = field(default_factory=list)
    missing_frames: list[str] = field(default_factory=list)
    missing_nodes: list[str] = field(default_factory=list)
    lifecycle_failures: list[str] = field(default_factory=list)
    condition_failures: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    detail: str = ""


@dataclass
class HealthSnapshot:
    overall_state: str
    revision: int
    components: list[HealthComponent]
    blocker_codes: list[str] = field(default_factory=list)
    blocker_messages: list[str] = field(default_factory=list)
    warning_codes: list[str] = field(default_factory=list)
    warning_messages: list[str] = field(default_factory=list)


def _severity(value: Any, default: str = ERROR) -> str:
    value = str(value or default).upper()
    return value if value in _RANK else default


def _as_observation(value: Any) -> TopicObservation:
    if isinstance(value, TopicObservation):
        return value
    if isinstance(value, Mapping):
        return TopicObservation(
            count=int(value.get("count", 0)),
            first_seen=value.get("first_seen"),
            last_seen=value.get("last_seen"),
            message_type=str(value.get("message_type", "")),
        )
    return TopicObservation()


class HealthEvaluator:
    """Evaluate a config-driven health contract for one main runtime mode."""

    def __init__(self, contract: Mapping[str, Any]):
        health = contract.get("health", contract)
        self.period_sec = float(health.get("period_sec", 1.0))
        self.components = list(health.get("components", []))
        self.revision = 0

    def evaluate(
        self,
        mode: str,
        observations: Mapping[str, Any] | None = None,
        *,
        now: float = 0.0,
        frames: set[str] | None = None,
        nodes: set[str] | None = None,
        lifecycle_states: Mapping[str, str] | None = None,
        conditions: Mapping[str, Any] | None = None,
    ) -> HealthSnapshot:
        observations = observations or {}
        frames = frames or set()
        nodes = nodes or set()
        lifecycle_states = lifecycle_states or {}
        conditions = conditions or {}
        self.revision += 1
        results: list[HealthComponent] = []

        for contract in self.components:
            component_id = str(contract["component_id"])
            active = mode in [str(item) for item in contract.get("required_in_modes", [])]
            optional = bool(contract.get("optional", False))
            required = active and not optional
            result = HealthComponent(
                component_id=component_id,
                display_name=str(contract.get("display_name", component_id)),
                required=required,
            )
            if not active:
                result.detail = "not_required_in_mode"
                results.append(result)
                continue

            checks: list[tuple[str, str, str]] = []
            topic_rates: list[float] = []
            topic_ages: list[float] = []
            topic_counts: list[int] = []
            topic_specs = contract.get("required_topics", [])
            if isinstance(topic_specs, Mapping):
                topic_specs = [topic_specs]
            for raw_spec in topic_specs:
                spec = {"name": raw_spec} if isinstance(raw_spec, str) else raw_spec
                name = str(spec["name"])
                observation = _as_observation(observations.get(name))
                present = observation.count > 0
                result.present = result.present or present
                topic_counts.append(observation.count)
                if present:
                    rate = observation.rate(now)
                    age = observation.age_at(now)
                    topic_rates.append(rate)
                    topic_ages.append(age)
                    if spec.get("expected_message_type") and observation.message_type:
                        if observation.message_type != str(spec["expected_message_type"]):
                            checks.append(("ERROR", name, "message_type_mismatch"))
                    if (
                        age > float(spec.get("max_age_sec", contract.get("max_age_sec", float("inf"))))
                        and not bool(spec.get("persistent", contract.get("persistent", False)))
                    ):
                        checks.append(("ERROR", name, "message_expired"))
                    min_rate = spec.get("min_rate_hz", contract.get("min_rate_hz"))
                    if min_rate is not None and rate < float(min_rate):
                        checks.append((_severity(spec.get("low_rate_severity", "WARN")), name, "rate_low"))
                    max_rate = spec.get("max_rate_hz", contract.get("max_rate_hz"))
                    if max_rate is not None and rate > float(max_rate):
                        checks.append((_severity(spec.get("high_rate_severity", "WARN")), name, "rate_high"))
                else:
                    checks.append((ERROR if required else WARN, name, "never_received"))
                    result.missing_topics.append(name)

            for frame in contract.get("required_frames", []):
                frame = str(frame)
                if frame not in frames:
                    checks.append((ERROR if required else WARN, frame, "frame_missing"))
                    result.missing_frames.append(frame)

            for node in contract.get("required_nodes", []):
                node = str(node)
                if node not in nodes:
                    checks.append((ERROR if required else WARN, node, "node_missing"))
                    result.missing_nodes.append(node)

            lifecycle_specs = contract.get("required_lifecycle_states", {})
            if isinstance(lifecycle_specs, list):
                lifecycle_specs = {str(item["node"]): str(item["state"]) for item in lifecycle_specs}
            for node, expected in lifecycle_specs.items():
                actual = lifecycle_states.get(str(node))
                if actual != str(expected):
                    checks.append((ERROR if required else WARN, str(node), f"lifecycle_{expected}"))
                    result.lifecycle_failures.append(f"{node}={actual or 'UNKNOWN'} (expected {expected})")

            for condition in contract.get("conditions", []):
                name = str(condition["name"])
                expected = condition.get("expected", True)
                if conditions.get(name) != expected:
                    severity = _severity(condition.get("severity", ERROR))
                    checks.append((severity, name, "condition_failed"))
                    result.condition_failures.append(name)

            result.observed_rate_hz = min(topic_rates) if topic_rates else 0.0
            result.message_age_sec = max(topic_ages) if topic_ages else float("inf")
            result.message_count = min(topic_counts) if topic_counts else 0
            result.present = result.present or bool(
                not topic_specs and (result.missing_frames == [] and result.missing_nodes == [])
            )
            for severity, subject, reason in checks:
                text = f"{subject}: {reason}"
                result.state = severity if _RANK[severity] > _RANK[result.state] else result.state
                if severity == WARN:
                    result.warnings.append(text)
                elif severity == ERROR:
                    result.errors.append(text)
            if not checks:
                result.state = OK
                result.detail = "all_contracts_satisfied"
            else:
                result.detail = "; ".join(result.errors + result.warnings)
            results.append(result)

        active_results = [
            item for item, contract in zip(results, self.components)
            if mode in [str(value) for value in contract.get("required_in_modes", [])]
        ]
        blockers = [item for item in active_results if item.required and item.state == ERROR]
        warnings = [item for item in active_results if item.state in (WARN, ERROR) and not item.required]
        if blockers:
            overall = ERROR
        elif any(item.state == ERROR for item in active_results):
            overall = ERROR
        elif any(item.state == WARN for item in active_results):
            overall = WARN
        elif any(item.state == UNKNOWN for item in active_results):
            overall = UNKNOWN
        else:
            overall = OK
        return HealthSnapshot(
            overall_state=overall,
            revision=self.revision,
            components=results,
            blocker_codes=[f"HEALTH_{item.component_id.upper()}" for item in blockers],
            blocker_messages=[item.detail for item in blockers],
            warning_codes=[f"HEALTH_{item.component_id.upper()}" for item in warnings],
            warning_messages=[item.detail for item in warnings],
        )
