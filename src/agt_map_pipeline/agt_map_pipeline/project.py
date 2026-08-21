from __future__ import annotations
from pathlib import Path
from typing import Mapping
import os, tempfile, uuid
import yaml

from .map_authority import MapAuthorityError, validate_map_authority_document

PROJECT_SCHEMA = "agt_map_project/v1"
PROJECT_STATES = {"NEW", "PREPARING", "WAITING_HUMAN_REVIEW", "REVIEW_IN_PROGRESS", "READY_TO_FREEZE", "FROZEN", "BLOCKED", "FAILED"}
STAGE_STATUSES = {"READY", "CANDIDATE", "HUMAN_REQUIRED", "BLOCKED", "FAILED", "FROZEN"}
LAYER_STATUSES = STAGE_STATUSES | {"CANDIDATE_UNBOUNDED", "BLOCKED_NO_ROWS"}

def _dirs(root: Path) -> None:
    for rel in ("source", "config", "layers/terrain", "layers/obstacle", "layers/navigation", "layers/structure", "layers/traversability", "evidence", "review", "revisions"):
        (root / rel).mkdir(parents=True, exist_ok=True)

def write_project(project_dir: Path | str, document: Mapping[str, object]) -> Path:
    root = Path(project_dir); root.mkdir(parents=True, exist_ok=True)
    validate_project_document(document)
    target = root / "project.yaml"
    fd, tmp = tempfile.mkstemp(prefix=".project.", suffix=".yaml", dir=root)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(dict(document), stream, sort_keys=False)
        os.replace(tmp, target)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
    return target

def create_project(project_dir: Path, *, source: dict, preset: dict, declared_frame_id: str, site_id: str | None) -> dict:
    _dirs(project_dir)
    doc = {"schema": PROJECT_SCHEMA, "project_id": str(uuid.uuid4()), "site_id": site_id, "project_state": "NEW", "source": source, "preset": preset, "frame": {"declared_frame_id": declared_frame_id, "verification": "UNVERIFIED"}, "map_authority": None, "stages": {}, "layers": {}, "human_review": {"required": ["frame", "navigation_overrides", "site_boundary", "row_aisle_candidates", "semantic_features"]}, "accepted_revision": None}
    write_project(project_dir, doc)
    return doc

def validate_project_document(document: Mapping[str, object]) -> None:
    if document.get("schema") != PROJECT_SCHEMA: raise ValueError("unsupported project schema")
    if document.get("project_state") not in PROJECT_STATES: raise ValueError("invalid project state")
    frame = document.get("frame")
    if not isinstance(frame, Mapping) or frame.get("verification") not in {"UNVERIFIED", "VERIFIED"}: raise ValueError("invalid frame verification")
    map_authority = document.get("map_authority")
    if map_authority is not None:
        if not isinstance(map_authority, Mapping):
            raise ValueError("invalid map authority binding")
        try:
            validate_map_authority_document(map_authority)
        except MapAuthorityError as exc:
            raise ValueError(f"invalid map authority binding: {exc}") from exc
    for name, stage in document.get("stages", {}).items():
        if stage.get("status") not in STAGE_STATUSES: raise ValueError(f"invalid stage status: {name}")
    for layer_id, layer in document.get("layers", {}).items():
        if layer.get("status") not in LAYER_STATUSES: raise ValueError(f"invalid layer status: {layer_id}")
        path = Path(str(layer.get("path", "")))
        if path.is_absolute() or ".." in path.parts: raise ValueError(f"layer path must be relative: {layer_id}")

def load_project(project_dir: Path | str) -> dict:
    root = Path(project_dir)
    with (root / "project.yaml").open(encoding="utf-8") as stream: doc = yaml.safe_load(stream)
    if not isinstance(doc, dict): raise ValueError("project manifest must be a mapping")
    validate_project_document(doc)
    return doc

def register_stage(document: dict, name: str, *, status: str, inputs_sha256: str, outputs: list[dict], message: str) -> None:
    if status not in STAGE_STATUSES: raise ValueError(f"invalid status: {status}")
    document.setdefault("stages", {})[name] = {"status": status, "inputs_sha256": inputs_sha256, "outputs": outputs, "message": message}

def register_layer(document: dict, layer_id: str, *, status: str, path: str, sha256: str, stage: str, parents: list[str]) -> None:
    if status not in LAYER_STATUSES: raise ValueError(f"invalid layer status: {status}")
    rel = Path(path)
    if rel.is_absolute() or ".." in rel.parts: raise ValueError("layer path must be relative")
    document.setdefault("layers", {})[layer_id] = {"status": status, "path": rel.as_posix(), "sha256": sha256, "stage": stage, "parents": parents}

