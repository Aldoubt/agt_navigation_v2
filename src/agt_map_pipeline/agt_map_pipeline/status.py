from __future__ import annotations
from pathlib import Path
from .project import load_project
from .hashing import file_sha256
from .map_authority import MapAuthorityError, verify_bound_map_authority

def _authority_block_code(exc: MapAuthorityError) -> str:
    message = str(exc).lower()
    if "hash mismatch" in message or "grid mismatch" in message or "does not match bound" in message:
        return "BLOCKED_V25_MAP_HASH_MISMATCH"
    if "missing" in message or "does not exist" in message:
        return "BLOCKED_V25_MAP_REVISION_INCOMPLETE"
    return "BLOCKED_V25_MAP_AUTHORITY_INVALID"

def build_project_status(project_dir):
    root = Path(project_dir).resolve(); doc = load_project(root)
    effective = doc["project_state"]; block = None
    source = doc["source"]
    authority = doc.get("map_authority")
    try:
        if file_sha256(source["absolute_path"]) != source["sha256"]:
            effective, block = "BLOCKED", "BLOCKED_SOURCE_HASH_MISMATCH"
        elif authority is not None:
            try:
                verify_bound_map_authority(authority)
            except MapAuthorityError as exc:
                effective, block = "BLOCKED", _authority_block_code(exc)
        if block is None:
            for layer_id, layer in doc.get("layers", {}).items():
                artifact = root / layer["path"]
                if not artifact.is_file() or (layer["sha256"] and file_sha256(artifact) != layer["sha256"]):
                    effective, block = "BLOCKED", "BLOCKED_LAYER_HASH_MISMATCH"
                    break
    except OSError:
        effective, block = "BLOCKED", "BLOCKED_SOURCE_MISSING"
    layers = {k: v["status"] for k, v in doc.get("layers", {}).items()}
    layers.setdefault("site_boundary", "HUMAN_REQUIRED"); layers.setdefault("semantic.headland", "HUMAN_REQUIRED")
    authority_summary = None
    if authority is not None:
        authority_summary = {"authority": authority.get("authority"), "status": authority.get("status")}
    result = {"schema": "agt_map_project_status/v1", "project_state": effective, "preset": doc["preset"]["name"], "source_sha256": source["sha256"], "map_authority": authority_summary, "layers": layers}
    if block:
        reasons = {
            "BLOCKED_SOURCE_HASH_MISMATCH": "source PCD identity changed; choose the intended source before resuming",
            "BLOCKED_SOURCE_MISSING": "source PCD is missing",
            "BLOCKED_LAYER_HASH_MISMATCH": "a registered project layer is missing or changed",
            "BLOCKED_V25_MAP_HASH_MISMATCH": "bound V25 map authority changed after binding; reselect the reviewed immutable revision",
            "BLOCKED_V25_MAP_REVISION_INCOMPLETE": "bound V25 map revision is incomplete or missing",
            "BLOCKED_V25_MAP_AUTHORITY_INVALID": "bound V25 map authority contract is invalid",
        }
        result["block_code"] = block; result["next_action"] = {"human_required": True, "command": None, "reason": reasons.get(block, "project prerequisites are blocked")}
    else:
        result["next_action"] = {"human_required": True, "command": f"agt-map review {root}", "reason": "site boundary and agricultural semantics require human review"}
    return result
