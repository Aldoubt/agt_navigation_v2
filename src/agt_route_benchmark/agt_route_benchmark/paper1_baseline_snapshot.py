from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

import yaml

from .formal_snapshot import validate_formal_site_snapshot
from .site_snapshot import create_site_snapshot, load_site_snapshot


P1_ACCEPTANCE_PURPOSE = "P1 formal site acceptance"


def write_p1_acceptance(
    output_path: Path | str,
    *,
    accepted_by: str = "Xuan Yang",
    accepted_at: str | None = None,
) -> Path:
    """Write the explicit author acceptance record used by the P1 snapshot."""
    accepted_by = str(accepted_by).strip()
    if not accepted_by:
        raise ValueError("accepted_by must be non-empty")
    accepted_at = accepted_at or datetime.now().astimezone().isoformat(timespec="seconds")
    document = {
        "schema_version": "1.0",
        "site_id": "greenhouse_01",
        "map_reliability_accepted": True,
        "semantic_correctness_accepted": True,
        "platform_geometry_accepted": True,
        "accepted_by": accepted_by,
        "accepted_at": accepted_at,
        "purpose": P1_ACCEPTANCE_PURPOSE,
        "not_e1_h_a4_human_effort_experiment": True,
    }
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def _validate_p1_acceptance(path: Path | str) -> dict[str, Any]:
    acceptance_path = Path(path)
    document = yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("P1 acceptance must be a mapping")
    if document.get("purpose") != P1_ACCEPTANCE_PURPOSE:
        raise ValueError("P1 acceptance purpose is invalid")
    if document.get("not_e1_h_a4_human_effort_experiment") is not True:
        raise ValueError("P1 acceptance must be separate from E1-H/A4")
    return document


def generate_p1_baseline_snapshot(
    *,
    pcd: Path | str,
    map_yaml: Path | str,
    semantic_map: Path | str,
    coverage_yaml: Path | str,
    platform_profile: Path | str,
    acceptance: Path | str,
    curation_manifest: Path | str,
    map_project: Path | str,
    output: Path | str,
) -> dict[str, Any]:
    """Generate a formal P1 snapshot from already accepted, immutable assets."""
    _validate_p1_acceptance(acceptance)
    return create_site_snapshot(
        "greenhouse_01",
        pcd,
        map_yaml,
        semantic_map,
        coverage_yaml,
        platform_profile,
        acceptance,
        curation_manifest_path=curation_manifest,
        map_project_path=map_project,
        output_path=output,
    )


def verify_p1_baseline_snapshot(path: Path | str) -> dict[str, Any]:
    """Verify snapshot checksum, all asset hashes, formal gates, and P1 purpose."""
    snapshot = load_site_snapshot(path, verify_assets=True)
    validate_formal_site_snapshot(snapshot)
    acceptance_path = snapshot["assets"]["acceptance"]["path"]
    acceptance = _validate_p1_acceptance(acceptance_path)
    return {
        "status": "PASS",
        "site_id": snapshot["site_id"],
        "snapshot_sha256": snapshot["snapshot_sha256"],
        "acceptance_purpose": acceptance["purpose"],
        "map_authority": snapshot["map_authority"],
        "curation_gate": snapshot["curation_gate"],
    }
