"""Ingest finalized managed mapping-session evidence into a PROCESSING workspace."""

from dataclasses import dataclass
from pathlib import Path
import os
import shutil
from typing import Any

import yaml

from .contracts import (
    AssetContractError,
    DatasetBinding,
    load_yaml_mapping,
    sha256_file,
    sha256_path_bundle,
)
from .workspace import compute_map_content_sha256


_ALLOWED_SESSION_STATES = {"CANDIDATE_READY", "REGISTERED"}


@dataclass(frozen=True)
class MappingSessionIngestResult:
    workspace_root: Path
    handoff_path: Path
    session_id: str
    derived_capture_bag_sha256: str

    def to_dict(self) -> dict[str, str]:
        return {
            "workspace_root": str(self.workspace_root),
            "handoff_path": str(self.handoff_path),
            "session_id": self.session_id,
            "derived_capture_bag_sha256": self.derived_capture_bag_sha256,
        }


def _atomic_yaml(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        yaml.safe_dump(value, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _copy_exact(source: Path, destination: Path) -> dict[str, str]:
    if not source.is_file():
        raise AssetContractError(
            "mapping_session_asset_missing", f"missing mapping-session asset: {source}"
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return {
        "path": str(destination),
        "sha256": sha256_file(destination),
    }


def _relative_record(root: Path, path: Path) -> dict[str, str]:
    return {
        "path": str(path.relative_to(root)),
        "sha256": sha256_file(path),
    }


def _validate_candidate_pair(candidate_yaml: Path, candidate_image: Path) -> None:
    metadata = load_yaml_mapping(candidate_yaml)
    image_value = str(metadata.get("image", "")).strip()
    if not image_value:
        raise AssetContractError(
            "mapping_session_candidate_invalid", "candidate map YAML has no image"
        )
    referenced = Path(image_value).expanduser()
    if not referenced.is_absolute():
        referenced = candidate_yaml.parent / referenced
    if referenced.resolve() != candidate_image.resolve():
        raise AssetContractError(
            "mapping_session_candidate_mismatch",
            "candidate map YAML does not reference the supplied candidate image",
        )
    if str(metadata.get("mode", "trinary")).lower() != "trinary":
        raise AssetContractError(
            "mapping_session_candidate_invalid", "candidate map mode must be trinary"
        )
    try:
        resolution = float(metadata["resolution"])
        origin = [float(value) for value in metadata["origin"]]
    except (KeyError, TypeError, ValueError) as exc:
        raise AssetContractError(
            "mapping_session_candidate_invalid",
            "candidate map resolution/origin metadata is invalid",
        ) from exc
    if resolution <= 0.0 or len(origin) != 3:
        raise AssetContractError(
            "mapping_session_candidate_invalid",
            "candidate map requires positive resolution and 3-element origin",
        )


def _validate_localization_pair(pcd: Path, processing_record: Path) -> None:
    if not pcd.is_file() or pcd.stat().st_size <= 0:
        raise AssetContractError(
            "mapping_session_pcd_invalid", "mapping-session localization PCD is missing or empty"
        )
    processing = load_yaml_mapping(processing_record)
    if str(processing.get("state", "")).lower() != "ready":
        raise AssetContractError(
            "mapping_session_processing_not_ready", "localization processing record must be ready"
        )
    expected = str(processing.get("pcd_sha256") or processing.get("map_hash") or "")
    actual = sha256_file(pcd)
    if expected != actual:
        raise AssetContractError(
            "mapping_session_pcd_hash_mismatch",
            "localization processing record does not match the supplied PCD",
        )


def ingest_mapping_session(
    manifest_path: str | Path,
    *,
    session_file: str | Path,
    candidate_map_yaml: str | Path,
    candidate_map_image: str | Path,
    localization_pcd: str | Path,
    processing_record: str | Path,
    derived_bag_directory: str | Path,
    session_id: str = "",
    source_bag_path: str | Path | None = None,
    candidate_report: str | Path | None = None,
) -> MappingSessionIngestResult:
    """Freeze one finalized MappingSession as pre-alignment derivation evidence.

    The captured map/PCD remain in the mapping-session frame. They are stored under
    processing/mapping_session and are intentionally *not* copied into canonical
    navigation/ or pointcloud/localization_map.pcd. A later alignment/materialize
    stage owns the session-frame -> canonical-map transform.
    """
    manifest_path = Path(manifest_path).expanduser().resolve()
    manifest = load_yaml_mapping(manifest_path)
    if str(manifest.get("state", "")).upper() == "READY":
        raise AssetContractError(
            "ready_map_immutable", "READY map content cannot accept new mapping-session evidence"
        )
    if str(manifest.get("state", "")).upper() not in {"PROCESSING", "DRAFT", "INVALID"}:
        raise AssetContractError(
            "mapping_session_workspace_state_invalid",
            "mapping-session ingestion requires a non-READY editable workspace",
        )

    root = manifest_path.parent
    session_path = Path(session_file).expanduser().resolve()
    candidate_yaml = Path(candidate_map_yaml).expanduser().resolve()
    candidate_image = Path(candidate_map_image).expanduser().resolve()
    pcd = Path(localization_pcd).expanduser().resolve()
    processing = Path(processing_record).expanduser().resolve()
    derived_bag = Path(derived_bag_directory).expanduser().resolve()

    for path in (session_path, candidate_yaml, candidate_image, pcd, processing):
        if not path.is_file():
            raise AssetContractError(
                "mapping_session_asset_missing", f"missing mapping-session asset: {path}"
            )
    if not derived_bag.is_dir() or not (derived_bag / "metadata.yaml").is_file():
        raise AssetContractError(
            "mapping_session_bag_invalid", "derived mapping-session bag is incomplete"
        )

    session = load_yaml_mapping(session_path)
    actual_session_id = str(session.get("session_id", "")).strip()
    requested_session_id = str(session_id).strip()
    if requested_session_id and actual_session_id != requested_session_id:
        raise AssetContractError(
            "mapping_session_id_mismatch", "session_file identity differs from requested session_id"
        )
    if not actual_session_id:
        raise AssetContractError("mapping_session_id_missing", "session_file has no session_id")
    if str(session.get("map_id", "")) != str(manifest.get("map_id", "")):
        raise AssetContractError(
            "mapping_session_map_mismatch", "mapping-session map_id differs from workspace map_id"
        )
    session_state = str(session.get("state", "")).upper()
    if session_state not in _ALLOWED_SESSION_STATES:
        raise AssetContractError(
            "mapping_session_not_finalized",
            "mapping-session evidence requires CANDIDATE_READY or REGISTERED state",
        )

    start_arguments = session.get("start_arguments") or {}
    platform_path_value = str(start_arguments.get("platform_profile", "")).strip()
    if not platform_path_value:
        raise AssetContractError(
            "mapping_session_platform_missing", "mapping session did not record platform_profile"
        )
    platform_path = Path(platform_path_value).expanduser().resolve()
    if not platform_path.is_file():
        raise AssetContractError(
            "mapping_session_platform_missing", "mapping-session platform profile is unavailable"
        )
    if sha256_file(platform_path) != str(manifest.get("platform_profile_sha256", "")):
        raise AssetContractError(
            "mapping_session_platform_mismatch",
            "mapping-session platform profile differs from workspace lineage",
        )

    dataset_relative = Path(str((manifest.get("source") or {}).get("dataset_binding", "")))
    dataset_path = root / dataset_relative
    dataset = DatasetBinding.from_file(dataset_path)
    if source_bag_path is not None:
        source_bag = Path(source_bag_path).expanduser().resolve()
        actual_source_hash = sha256_path_bundle(source_bag)
        if actual_source_hash != dataset.bag_sha256:
            raise AssetContractError(
                "source_bag_hash_mismatch",
                "replayed source bag differs from the workspace Dataset binding",
            )
    else:
        actual_source_hash = dataset.bag_sha256

    _validate_candidate_pair(candidate_yaml, candidate_image)
    _validate_localization_pair(pcd, processing)

    destination = root / "processing" / "mapping_session"
    if destination.exists():
        raise AssetContractError(
            "mapping_session_already_ingested",
            "workspace already contains mapping-session evidence; create a new map version to replay again",
        )
    candidate_dir = destination / "candidate"
    localization_dir = destination / "localization"
    evidence_dir = destination / "evidence"
    candidate_dir.mkdir(parents=True)
    localization_dir.mkdir(parents=True)
    evidence_dir.mkdir(parents=True)

    session_copy = evidence_dir / "session.yaml"
    candidate_yaml_copy = candidate_dir / candidate_yaml.name
    candidate_image_copy = candidate_dir / candidate_image.name
    pcd_copy = localization_dir / pcd.name
    processing_copy = localization_dir / processing.name
    shutil.copy2(session_path, session_copy)
    shutil.copy2(candidate_yaml, candidate_yaml_copy)
    shutil.copy2(candidate_image, candidate_image_copy)
    shutil.copy2(pcd, pcd_copy)
    shutil.copy2(processing, processing_copy)

    report_copy = None
    report_source = (
        Path(candidate_report).expanduser().resolve()
        if candidate_report is not None
        else candidate_yaml.parent / "comparison_report.json"
    )
    if report_source.is_file():
        report_copy = candidate_dir / report_source.name
        shutil.copy2(report_source, report_copy)

    derived_bag_hash = sha256_path_bundle(derived_bag)
    handoff = {
        "schema_version": 1,
        "session_id": actual_session_id,
        "session_state": session_state,
        "map_id": str(manifest.get("map_id", "")),
        "map_version_id": str(manifest.get("map_version_id", "")),
        "frame_semantics": {
            "source_frame": "mapping_session",
            "canonical_frame": "map",
            "materialized": False,
        },
        "source_dataset": {
            "dataset_id": dataset.dataset_id,
            "bag_sha256": actual_source_hash,
        },
        "derived_capture_bag": {
            "external_path": str(derived_bag),
            "sha256": derived_bag_hash,
        },
        "artifacts": {
            "session_file": _relative_record(root, session_copy),
            "candidate_map_yaml": _relative_record(root, candidate_yaml_copy),
            "candidate_map_image": _relative_record(root, candidate_image_copy),
            "localization_pcd": _relative_record(root, pcd_copy),
            "processing_record": _relative_record(root, processing_copy),
        },
    }
    if report_copy is not None:
        handoff["artifacts"]["candidate_report"] = _relative_record(root, report_copy)

    handoff_path = destination / "handoff.yaml"
    _atomic_yaml(handoff_path, handoff)

    derivation = dict(manifest.get("derivation") or {})
    derivation["mapping_session_handoff"] = str(handoff_path.relative_to(root))
    derivation["mapping_session_handoff_sha256"] = sha256_file(handoff_path)
    manifest["derivation"] = derivation

    assets = dict(manifest.get("assets") or {})
    for asset_id, path in (
        ("mapping_session_handoff", handoff_path),
        ("mapping_session_record", session_copy),
        ("mapping_session_candidate_yaml", candidate_yaml_copy),
        ("mapping_session_candidate_pgm", candidate_image_copy),
        ("mapping_session_localization_pcd", pcd_copy),
        ("mapping_session_processing_record", processing_copy),
    ):
        assets[asset_id] = _relative_record(root, path)
    if report_copy is not None:
        assets["mapping_session_candidate_report"] = _relative_record(root, report_copy)
    manifest["assets"] = assets
    manifest["state"] = "PROCESSING"
    manifest["map_content_sha256"] = compute_map_content_sha256(manifest)
    _atomic_yaml(manifest_path, manifest)

    return MappingSessionIngestResult(
        workspace_root=root,
        handoff_path=handoff_path,
        session_id=actual_session_id,
        derived_capture_bag_sha256=derived_bag_hash,
    )
