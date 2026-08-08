"""Non-destructive Route Asset tuning helpers."""

from dataclasses import replace
import math
from pathlib import Path
import shutil

from .contracts import AssetContractError, load_yaml_mapping, sha256_file
from .route_asset import load_route_csv, write_route_csv, write_route_manifest


def apply_route_tuning(
    route_dir: str | Path,
    tuning_path: str | Path,
    *,
    new_revision: int,
) -> Path:
    route_dir = Path(route_dir).expanduser().resolve()
    tuning_path = Path(tuning_path).expanduser().resolve()
    manifest = load_yaml_mapping(route_dir / "route.yaml")
    tuning = load_yaml_mapping(tuning_path)
    base_hash = str(tuning.get("base_route_sha256", ""))
    if base_hash != sha256_file(route_dir / "route.csv"):
        raise AssetContractError("tuning_base_hash_mismatch", "tuning base_route_sha256 does not match route.csv")
    if int(new_revision) <= int(manifest.get("revision", 0)):
        raise AssetContractError("tuning_revision_invalid", "new revision must be greater than base revision")

    samples = load_route_csv(route_dir / "route.csv")
    operations = tuning.get("operations") or []
    if not isinstance(operations, list):
        raise AssetContractError("tuning_operations_invalid", "tuning operations must be a list")
    for operation in operations:
        if not isinstance(operation, dict):
            raise AssetContractError("tuning_operation_invalid", "each tuning operation must be a mapping")
        kind = str(operation.get("type", ""))
        segment_id = str(operation.get("segment_id", ""))
        if not segment_id:
            raise AssetContractError("tuning_segment_missing", "tuning operation needs segment_id")
        matched = [index for index, sample in enumerate(samples) if sample.segment_id == segment_id]
        if not matched:
            raise AssetContractError("tuning_segment_unknown", f"unknown route segment: {segment_id}")
        if kind == "speed_scale":
            scale = float(operation.get("value", 0.0))
            if not math.isfinite(scale) or scale <= 0.0:
                raise AssetContractError("tuning_speed_scale_invalid", "speed_scale must be positive")
            for index in matched:
                samples[index] = replace(samples[index], v_ref=samples[index].v_ref * scale)
        elif kind == "lateral_offset":
            offset = float(operation.get("value_m", 0.0))
            if not math.isfinite(offset):
                raise AssetContractError("tuning_offset_invalid", "lateral_offset must be finite")
            for index in matched:
                sample = samples[index]
                samples[index] = replace(
                    sample,
                    x=sample.x - math.sin(sample.yaw) * offset,
                    y=sample.y + math.cos(sample.yaw) * offset,
                )
        else:
            raise AssetContractError("tuning_type_unsupported", f"unsupported tuning operation: {kind}")

    new_dir = route_dir.parent / str(int(new_revision))
    if new_dir.exists():
        raise AssetContractError("route_revision_exists", f"route revision already exists: {new_dir}")
    new_dir.mkdir(parents=True)
    shutil.copy2(route_dir / "policy.yaml", new_dir / "policy.yaml")
    shutil.copy2(tuning_path, new_dir / "tuning.yaml")
    write_route_csv(new_dir / "route.csv", samples)

    updated = dict(manifest)
    updated["revision"] = int(new_revision)
    updated["route_csv_sha256"] = sha256_file(new_dir / "route.csv")
    updated["policy_binding"] = {
        "path": "policy.yaml",
        "sha256": sha256_file(new_dir / "policy.yaml"),
    }
    updated.pop("feasibility_report_sha256", None)
    updated.pop("preview_sha256", None)
    updated["status"] = "DRAFT"
    updated["tuning"] = {
        "path": "tuning.yaml",
        "sha256": sha256_file(new_dir / "tuning.yaml"),
        "base_revision": int(manifest.get("revision", 0)),
        "base_route_sha256": base_hash,
    }
    write_route_manifest(new_dir / "route.yaml", updated)
    return new_dir
