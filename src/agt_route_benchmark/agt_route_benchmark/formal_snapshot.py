from __future__ import annotations

from typing import Any, Mapping


def validate_formal_site_snapshot(snapshot: Mapping[str, Any]) -> Mapping[str, Any]:
    """Reject development/legacy snapshots before Paper I formal execution."""
    if not isinstance(snapshot, Mapping):
        raise ValueError("formal site snapshot must be a mapping")
    if str(snapshot.get("curation_gate", "")) != "ACCEPTED_REPLAY_CLEAN":
        raise ValueError("formal curation gate must be ACCEPTED_REPLAY_CLEAN")
    assets = snapshot.get("assets")
    if not isinstance(assets, Mapping) or "curation_manifest" not in assets:
        raise ValueError("formal curation manifest asset is required")
    curation_asset = assets.get("curation_manifest")
    if not isinstance(curation_asset, Mapping) or len(str(curation_asset.get("sha256", ""))) != 64:
        raise ValueError("formal curation manifest asset hash is invalid")
    acceptance = snapshot.get("acceptance")
    if not isinstance(acceptance, Mapping):
        raise ValueError("formal snapshot acceptance record is missing")
    for key in (
        "map_reliability_accepted",
        "semantic_correctness_accepted",
        "platform_geometry_accepted",
    ):
        if acceptance.get(key) is not True:
            raise ValueError(f"{key} must be true for formal execution")
    return snapshot
