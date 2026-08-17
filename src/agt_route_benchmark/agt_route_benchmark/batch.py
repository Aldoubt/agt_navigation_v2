from __future__ import annotations

from dataclasses import dataclass, field
import csv
import json
from pathlib import Path
from typing import Mapping, Any, Iterable

from .contracts import P2P_PLANNERS, MISSION_PLANNERS, SCENARIO_IDS


@dataclass(frozen=True)
class MatrixCell:
    scenario_id: str
    planner_id: str
    status: str
    metrics: Mapping[str, Any] = field(default_factory=dict)


def canonical_expected_cells() -> tuple[tuple[str, str], ...]:
    """Return the frozen Paper I cells without meaningless planner/scenario pairs.

    S01-S05 are point-to-point constraint probes and are evaluated with the four
    P2P planner baselines. S06 is the agricultural mission and is evaluated with
    manual-waypoints+best-P2P, Fields2Cover, and the proposed method.
    """
    p2p_scenarios = SCENARIO_IDS[:-1]
    mission_scenarios = (SCENARIO_IDS[-1],)
    return tuple(
        [(scenario, planner) for scenario in p2p_scenarios for planner in P2P_PLANNERS]
        + [(scenario, planner) for scenario in mission_scenarios for planner in MISSION_PLANNERS]
    )


def planner_outcome_counts(cells: Iterable[MatrixCell]) -> dict[str, int]:
    """Count planner outcomes without treating invalid inputs as planner trials."""
    cells = list(cells)
    invalid_scenario_count = sum(cell.status == "INVALID_SCENARIO" for cell in cells)
    evaluated = [cell for cell in cells if cell.status != "INVALID_SCENARIO"]
    planner_success_count = sum(cell.status == "OK" for cell in evaluated)
    return {
        "planner_evaluated_count": len(evaluated),
        "planner_success_count": planner_success_count,
        "planner_failure_count": len(evaluated) - planner_success_count,
        "invalid_scenario_count": invalid_scenario_count,
    }


def validate_batch_completeness(
    cells: Iterable[MatrixCell],
    expected_cells: Iterable[tuple[str, str]],
    *,
    formal: bool,
) -> None:
    cells = list(cells)
    expected = tuple(expected_cells)
    by_key = {(cell.scenario_id, cell.planner_id): cell for cell in cells}
    if len(by_key) != len(cells):
        raise ValueError("duplicate matrix cells are not allowed")
    expected_set = set(expected)
    missing = [key for key in expected if key not in by_key]
    if missing:
        raise ValueError(f"missing matrix cells: {missing}")
    unexpected = sorted(set(by_key) - expected_set)
    if formal and unexpected:
        raise ValueError(f"formal matrix contains unexpected cells: {unexpected}")
    if formal:
        skipped = [key for key in expected if by_key[key].status == "SKIPPED_DEPENDENCY"]
        if skipped:
            raise ValueError(f"formal matrix contains skipped dependency cells: {skipped}")


def validate_canonical_batch_completeness(cells: Iterable[MatrixCell], *, formal: bool) -> None:
    validate_batch_completeness(cells, canonical_expected_cells(), formal=formal)


def write_comparison_summary(cells, output_dir: Path | str) -> tuple[Path, Path]:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    cells = list(cells)
    metric_keys = sorted({key for cell in cells for key in cell.metrics})
    rows = [
        {
            "scenario_id": cell.scenario_id,
            "planner_id": cell.planner_id,
            "status": cell.status,
            **{key: cell.metrics.get(key) for key in metric_keys},
        }
        for cell in cells
    ]
    csv_path = output_dir / "comparison.csv"
    json_path = output_dir / "comparison.json"
    fields = ["scenario_id", "planner_id", "status", *metric_keys]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    json_path.write_text(
        json.dumps(rows, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return csv_path, json_path
