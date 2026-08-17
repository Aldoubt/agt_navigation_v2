from __future__ import annotations

import json
from pathlib import Path


def load_site_snapshot_identity(path: Path | str) -> str:
    source = Path(path).expanduser().resolve()
    data = json.loads(source.read_text(encoding="utf-8"))
    value = str(data.get("snapshot_sha256", "")) if isinstance(data, dict) else ""
    if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
        raise ValueError("site snapshot must contain a valid 64-hex snapshot_sha256")
    return value


def select_result_run(
    result_root: Path | str,
    site_id: str,
    scenario_id: str,
    planner_id: str,
    *,
    formal: bool,
    site_snapshot_sha256: str | None = None,
) -> Path | None:
    base = Path(result_root).expanduser().resolve() / site_id / scenario_id
    if not base.is_dir():
        return None
    for candidate in reversed(sorted(base.glob(f"{planner_id}-*"))):
        report_path = candidate / "planner_report.json"
        metrics_path = candidate / "metrics.json"
        manifest_path = candidate / "experiment_manifest.json"
        if not report_path.is_file() or not metrics_path.is_file():
            continue
        if not formal:
            return candidate
        if site_snapshot_sha256 is None or not manifest_path.is_file():
            continue
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(manifest, dict) or manifest.get("formal") is not True:
            continue
        if str(manifest.get("site_id", site_id)) != site_id:
            continue
        if str(manifest.get("scenario_id", scenario_id)) != scenario_id:
            continue
        if str(manifest.get("planner_id", planner_id)) != planner_id:
            continue
        metadata = manifest.get("metadata")
        if not isinstance(metadata, dict):
            continue
        if str(metadata.get("site_snapshot_sha256", "")) != site_snapshot_sha256:
            continue
        return candidate
    return None
