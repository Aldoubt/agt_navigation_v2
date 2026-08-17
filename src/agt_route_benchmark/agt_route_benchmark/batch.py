from __future__ import annotations
from dataclasses import dataclass, field
import csv
import json
from pathlib import Path
from typing import Mapping, Any


@dataclass(frozen=True)
class MatrixCell:
    scenario_id: str
    planner_id: str
    status: str
    metrics: Mapping[str, Any] = field(default_factory=dict)


def validate_batch_completeness(cells, scenario_ids, planner_ids, *, formal: bool) -> None:
    by_key = {(c.scenario_id, c.planner_id): c for c in cells}
    missing = [(s, p) for s in scenario_ids for p in planner_ids if (s, p) not in by_key]
    if missing:
        raise ValueError(f"missing matrix cells: {missing}")
    if formal:
        skipped = [key for key, c in by_key.items() if c.status == "SKIPPED_DEPENDENCY"]
        if skipped:
            raise ValueError(f"formal matrix contains skipped dependency cells: {skipped}")


def write_comparison_summary(cells, output_dir: Path | str) -> tuple[Path, Path]:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    metric_keys = sorted({k for cell in cells for k in cell.metrics})
    rows = [
        {"scenario_id": cell.scenario_id, "planner_id": cell.planner_id, "status": cell.status, **{k: cell.metrics.get(k) for k in metric_keys}}
        for cell in cells
    ]
    csv_path = output_dir / "comparison.csv"
    json_path = output_dir / "comparison.json"
    fields = ["scenario_id", "planner_id", "status", *metric_keys]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    json_path.write_text(json.dumps(rows, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    return csv_path, json_path
