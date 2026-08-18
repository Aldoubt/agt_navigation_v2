from __future__ import annotations
from pathlib import Path
from .project import load_project
from .hashing import file_sha256

def build_project_status(project_dir):
    root = Path(project_dir).resolve(); doc = load_project(root)
    effective = doc["project_state"]; block = None
    source = doc["source"]
    try:
        if file_sha256(source["absolute_path"]) != source["sha256"]:
            effective, block = "BLOCKED", "BLOCKED_SOURCE_HASH_MISMATCH"
    except OSError:
        effective, block = "BLOCKED", "BLOCKED_SOURCE_MISSING"
    layers = {k: v["status"] for k, v in doc.get("layers", {}).items()}
    layers.setdefault("site_boundary", "HUMAN_REQUIRED"); layers.setdefault("semantic.headland", "HUMAN_REQUIRED")
    result = {"schema": "agt_map_project_status/v1", "project_state": effective, "preset": doc["preset"]["name"], "source_sha256": source["sha256"], "layers": layers}
    if block:
        result["block_code"] = block; result["next_action"] = {"human_required": True, "command": None, "reason": "source PCD identity changed; choose the intended source before resuming"}
    else:
        result["next_action"] = {"human_required": True, "command": f"agt-map review {root}", "reason": "site boundary and agricultural semantics require human review"}
    return result
