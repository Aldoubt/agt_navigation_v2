"""One-click review package helpers for the V25 Map Workbench."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Mapping


REVIEW_LAYER_EXPORTS = (
    ("01_pointcloud.png", None),
    ("02_ground_relative.png", "final"),
    ("03_row_structural_band.png", "row_structural_band"),
    ("04_aisle_geometric_envelope.png", "aisle_geometric_envelope"),
    ("05_aisle_geometric_centerline.png", "aisle_geometric_centerline"),
    ("06_safe_aisle_centerline.png", "aisle_centerline"),
    ("07_soft_occupied_candidate.png", "formal_soft_occupied"),
    ("08_soft_occupied_recovered.png", "formal_soft_recovered"),
    ("09_formal_generated.png", "formal_generated"),
    ("10_formal_accepted.png", "formal_accepted"),
    ("11_generated_accepted_diff.png", "formal_diff"),
)


def git_head(cwd: str | Path) -> str | None:
    """Return the current commit without making review export depend on Git."""

    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(Path(cwd)),
            check=True,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    value = completed.stdout.strip()
    return value or None


def _summary_lines(value, *, prefix: str = "") -> list[str]:
    lines: list[str] = []
    if isinstance(value, Mapping):
        for key, child in value.items():
            label = f"{prefix}.{key}" if prefix else str(key)
            if isinstance(child, Mapping):
                lines.append(f"[{label}]")
                lines.extend(_summary_lines(child, prefix=label))
            elif isinstance(child, (list, tuple)):
                lines.append(f"{label}: {json.dumps(child, ensure_ascii=False)}")
            else:
                lines.append(f"{label}: {child}")
        return lines
    lines.append(f"{prefix}: {value}")
    return lines


def write_review_summary(
    destination: str | Path,
    summary: Mapping[str, object],
) -> tuple[Path, Path]:
    """Write machine-readable and quick human-readable review metadata."""

    root = Path(destination).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    json_path = root / "review_summary.json"
    text_path = root / "review_summary.txt"
    document = dict(summary)
    json_path.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    text_path.write_text("\n".join(_summary_lines(document)) + "\n", encoding="utf-8")
    return json_path, text_path
