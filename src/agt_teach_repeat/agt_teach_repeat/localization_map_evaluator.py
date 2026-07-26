"""Integrity and repeated-relocation grading for a teach-bound localization map."""

import argparse
import json
import math
from pathlib import Path
import statistics

import yaml
from agt_ui_bridge.map_transform import MapGeometry

from .path_io import (
    load_manifest,
    load_reference_path,
    manifest_reference_path,
    resolve_asset,
    sha256_file,
)


LOCALIZATION_FIELDS = (
    "success",
    "state",
    "error_code",
    "has_converged",
    "ambiguous_result",
    "fitness_score",
    "overlap_ratio",
    "inlier_ratio",
    "ambiguity_score",
    "translation_innovation",
    "yaw_innovation",
    "runtime_ms",
    "tested_candidates",
    "total_candidates",
)


def pcd_point_count(path):
    with open(path, "rb") as stream:
        header = stream.read(1024 * 1024).split(b"DATA", 1)[0].decode("ascii", errors="strict")
    values = {}
    for line in header.splitlines():
        parts = line.strip().split()
        if len(parts) >= 2:
            values[parts[0].upper()] = parts[1:]
    if "POINTS" in values:
        return int(values["POINTS"][0])
    return int(values.get("WIDTH", [0])[0]) * int(
        values.get("HEIGHT", [1])[0]
    )


def check_map_integrity(manifest_path, minimum_pcd_points=1000):
    manifest_path, manifest = load_manifest(manifest_path)
    map_yaml = resolve_asset(manifest_path, manifest["map"]["map_yaml"])
    pcd = resolve_asset(manifest_path, manifest["map"]["localization_pcd"])
    record_path = resolve_asset(manifest_path, manifest["map"]["processing_record"])
    errors = []
    assets = (
        ("map_yaml", map_yaml),
        ("localization_pcd", pcd),
        ("processing_record", record_path),
    )
    for name, path in assets:
        if not path.is_file():
            errors.append(f"{name}_missing")
    point_count = 0
    if not errors:
        try:
            MapGeometry.from_nav2_yaml(map_yaml)
            load_reference_path(
                manifest_reference_path(manifest_path, manifest),
                expected_demo_id=manifest["demo_id"],
            )
            record = yaml.safe_load(record_path.read_text(encoding="utf-8")) or {}
            if record.get("state") != "ready":
                errors.append("processing_record_not_ready")
            map_file = str(record.get("map_file", ""))
            if map_file and (record_path.parent / map_file).resolve() != pcd:
                errors.append("processing_record_pcd_mismatch")
            actual_hash = sha256_file(pcd)
            if actual_hash != manifest["map"].get("localization_pcd_sha256"):
                errors.append("manifest_pcd_hash_mismatch")
            recorded_hash = str(
                record.get("pcd_sha256") or record.get("map_hash") or ""
            )
            if recorded_hash and recorded_hash != actual_hash:
                errors.append("processing_record_hash_mismatch")
            if sha256_file(map_yaml) != manifest["map"].get("map_yaml_sha256"):
                errors.append("manifest_map_hash_mismatch")
            point_count = pcd_point_count(pcd)
            if point_count < int(minimum_pcd_points):
                errors.append("pcd_point_count_too_small")
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            UnicodeError,
            yaml.YAMLError,
        ):
            errors.append("map_asset_unreadable")
    return {
        "valid": not errors,
        "errors": sorted(set(errors)),
        "map_id": manifest["map"].get("map_id", ""),
        "pcd_point_count": point_count,
        "minimum_pcd_points": int(minimum_pcd_points),
    }


def normalize_localization_sample(value):
    return {
        field: value.get(field) if value.get(field) is not None else None
        for field in LOCALIZATION_FIELDS
    }


def evaluate_localization(
    samples, thresholds, *, integrity_valid=True, field_validated=False
):
    normalized = [normalize_localization_sample(item) for item in samples]
    successes = [item for item in normalized if item["success"] is True]
    ambiguous = sum(item.get("ambiguous_result") is True for item in normalized)
    wrong_accepts = sum(item.get("wrong_accept") is True for item in samples)
    lost = sum(
        str(item.get("state", "")).upper() in {"LOST", "6"}
        for item in normalized
    )
    times = [
        float(item["runtime_ms"]) / 1000.0
        for item in normalized
        if item.get("runtime_ms") is not None
        and math.isfinite(float(item["runtime_ms"]))
    ]
    attempts = len(normalized)
    summary = {
        "attempts": attempts,
        "successes": len(successes),
        "success_rate": len(successes) / attempts if attempts else 0.0,
        "ambiguous_results": ambiguous,
        "wrong_accepts": wrong_accepts,
        "tracking_lost_count": lost,
        "median_relocalization_time_s": statistics.median(times) if times else None,
        "samples": normalized,
    }
    passes = (
        integrity_valid
        and attempts >= int(thresholds["minimum_attempts"])
        and summary["success_rate"] >= float(thresholds["minimum_success_rate"])
        and ambiguous <= int(thresholds["maximum_ambiguous_results"])
        and wrong_accepts <= int(thresholds["maximum_wrong_accepts"])
        and lost <= int(thresholds["maximum_tracking_lost_count"])
        and summary["median_relocalization_time_s"] is not None
        and summary["median_relocalization_time_s"]
        <= float(thresholds["maximum_median_relocalization_time_s"])
    )
    if not integrity_valid:
        grade = "INVALID"
    elif not passes:
        grade = "OFFLINE_ONLY"
    elif field_validated:
        grade = "FIELD_VALIDATED"
    else:
        grade = "FIELD_CANDIDATE"
    summary["grade"] = grade
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description="Grade a teach-bound localization map")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--samples-json", default="")
    parser.add_argument("--config", default="")
    parser.add_argument("--minimum-pcd-points", type=int, default=1000)
    parser.add_argument("--field-validated", action="store_true")
    parser.add_argument("--output", default="")
    args = parser.parse_args(argv)
    defaults = {
        "minimum_attempts": 3,
        "minimum_success_rate": 0.80,
        "maximum_ambiguous_results": 0,
        "maximum_wrong_accepts": 0,
        "maximum_tracking_lost_count": 0,
        "maximum_median_relocalization_time_s": 5.0,
    }
    if args.config:
        document = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
        parameters = document.get("/**", {}).get("ros__parameters", document)
        defaults.update(parameters.get("localization_evaluation", {}))
    samples = (
        json.loads(Path(args.samples_json).read_text(encoding="utf-8"))
        if args.samples_json
        else []
    )
    integrity = check_map_integrity(args.manifest, args.minimum_pcd_points)
    result = {
        "map_integrity": integrity,
        "localization_summary": evaluate_localization(
            samples,
            defaults,
            integrity_valid=integrity["valid"],
            field_validated=args.field_validated,
        ),
    }
    content = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        from .path_io import _atomic_bytes

        _atomic_bytes(args.output, content.encode("utf-8"))
    print(content, end="")


if __name__ == "__main__":
    main()
