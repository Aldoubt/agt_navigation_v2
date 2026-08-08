"""Machine-readable, non-destructive point-cloud cleaning records."""

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping
import json
import yaml

from .contracts import AssetContractError, sha256_file


ALLOWED_OPERATIONS = {"crop_box", "crop_polygon", "delete_polygon", "height_range", "voxel_downsample", "sor", "ground_separation", "stable_structure_selection"}


def append_cleaning_operation(record_path: str | Path, *, operation: str, parameters: Mapping[str, Any], input_path: str | Path, output_path: str | Path, operator_note: str = "") -> dict[str, Any]:
    if operation not in ALLOWED_OPERATIONS:
        raise AssetContractError("cleaning_operation_invalid", f"unsupported cleaning operation: {operation}")
    source, result = Path(input_path).resolve(), Path(output_path).resolve()
    if not source.is_file() or not result.is_file():
        raise AssetContractError("cleaning_asset_missing", "cleaning input and output files are required")
    path = Path(record_path)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) if path.is_file() else {}
    if not isinstance(data, dict):
        raise AssetContractError("cleaning_record_invalid", "cleaning record must be a YAML mapping")
    operations = list(data.get("operations") or [])
    operations.append({"operation": operation, "parameters": dict(parameters), "input": {"path": str(source), "sha256": sha256_file(source)}, "output": {"path": str(result), "sha256": sha256_file(result)}, "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"), "operator_note": operator_note})
    data.update({"schema_version": 1, "state": "RECORDED", "operations": operations})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    report = {"schema_version": 1, "status": "PASS", "operation_count": len(operations), "latest_output_sha256": sha256_file(result)}
    report_path = path.with_name("cleaning_report.json")
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
