"""Machine-readable contracts for datasets, derivation recipes, and route policy."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re
from typing import Any, Mapping

import yaml


_HASH_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
_ALLOWED_PURPOSES = {"EVALUATION", "OPERATIONAL"}
_ALLOWED_ROW_INTERPRETATIONS = {"direct_swaths", "crop_centerlines"}


class AssetContractError(ValueError):
    """Stable validation failure for operator-facing offline asset tooling."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = str(code)


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return "sha256:" + digest.hexdigest()


def sha256_path_bundle(path: str | Path) -> str:
    """Hash one file or a directory tree deterministically.

    A directory digest includes every regular file under the directory, sorted by
    POSIX relative path. Each record contributes the relative path, file size and
    file-content SHA256. Symlinks are rejected so a Dataset binding cannot silently
    depend on data outside the managed bag directory.
    """
    root = Path(path).expanduser().resolve()
    if root.is_file():
        return sha256_file(root)
    if not root.is_dir():
        raise AssetContractError("bundle_path_missing", f"bundle path does not exist: {root}")

    files = []
    for candidate in root.rglob("*"):
        if candidate.is_symlink():
            raise AssetContractError(
                "bundle_symlink_forbidden",
                f"bundle contains a symlink: {candidate.relative_to(root)}",
            )
        if candidate.is_file():
            files.append(candidate)
    if not files:
        raise AssetContractError("bundle_empty", f"bundle has no files: {root}")

    digest = hashlib.sha256()
    for candidate in sorted(files, key=lambda item: item.relative_to(root).as_posix()):
        relative = candidate.relative_to(root).as_posix().encode("utf-8")
        file_hash = sha256_file(candidate).removeprefix("sha256:").encode("ascii")
        size = str(candidate.stat().st_size).encode("ascii")
        digest.update(relative)
        digest.update(b"\0")
        digest.update(size)
        digest.update(b"\0")
        digest.update(file_hash)
        digest.update(b"\n")
    return "sha256:" + digest.hexdigest()


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise AssetContractError("yaml_not_mapping", f"{path} must contain a YAML mapping")
    return value


def _require_text(data: Mapping[str, Any], key: str, *, code: str) -> str:
    value = str(data.get(key, "")).strip()
    if not value:
        raise AssetContractError(code, f"missing required field: {key}")
    return value


def _require_hash(value: Any, *, code: str, field: str) -> str:
    text = str(value or "")
    if not _HASH_RE.fullmatch(text):
        raise AssetContractError(code, f"{field} must be sha256:<64 lowercase hex>")
    return text


@dataclass(frozen=True)
class DatasetBinding:
    dataset_id: str
    site_id: str
    epoch_id: str
    purpose: str
    bag_path: str
    bag_sha256: str
    platform_id: str
    platform_profile_sha256: str
    calibration_id: str
    calibration_sha256: str

    @classmethod
    def from_file(cls, path: str | Path) -> "DatasetBinding":
        return cls.from_dict(load_yaml_mapping(path))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "DatasetBinding":
        bag = data.get("bag")
        platform = data.get("platform")
        calibration = data.get("calibration")
        if not isinstance(bag, Mapping):
            raise AssetContractError("dataset_bag_missing", "dataset bag mapping is required")
        if not isinstance(platform, Mapping):
            raise AssetContractError("dataset_platform_missing", "dataset platform mapping is required")
        if not isinstance(calibration, Mapping):
            raise AssetContractError("dataset_calibration_missing", "dataset calibration mapping is required")
        purpose = _require_text(data, "purpose", code="dataset_purpose_missing").upper()
        if purpose not in _ALLOWED_PURPOSES:
            raise AssetContractError(
                "dataset_purpose_invalid",
                f"purpose must be one of {sorted(_ALLOWED_PURPOSES)}",
            )
        return cls(
            dataset_id=_require_text(data, "dataset_id", code="dataset_id_missing"),
            site_id=_require_text(data, "site_id", code="dataset_site_missing"),
            epoch_id=_require_text(data, "epoch_id", code="dataset_epoch_missing"),
            purpose=purpose,
            bag_path=_require_text(bag, "path", code="dataset_bag_path_missing"),
            bag_sha256=_require_hash(
                bag.get("sha256"), code="dataset_bag_hash_invalid", field="bag.sha256"
            ),
            platform_id=_require_text(
                platform, "profile_id", code="dataset_platform_id_missing"
            ),
            platform_profile_sha256=_require_hash(
                platform.get("profile_sha256"),
                code="dataset_platform_hash_invalid",
                field="platform.profile_sha256",
            ),
            calibration_id=_require_text(
                calibration, "calibration_id", code="dataset_calibration_id_missing"
            ),
            calibration_sha256=_require_hash(
                calibration.get("calibration_sha256"),
                code="dataset_calibration_hash_invalid",
                field="calibration.calibration_sha256",
            ),
        )

    def resolve_bag_path(self, binding_path: str | Path) -> Path:
        bag = Path(self.bag_path).expanduser()
        if not bag.is_absolute():
            bag = (Path(binding_path).resolve().parent / bag).resolve()
        return bag

    def verify_bag(self, binding_path: str | Path) -> Path:
        bag = self.resolve_bag_path(binding_path)
        if not bag.exists():
            raise AssetContractError("dataset_bag_missing", f"bound bag does not exist: {bag}")
        if bag.is_dir() and not (bag / "metadata.yaml").is_file():
            raise AssetContractError(
                "dataset_bag_metadata_missing",
                "rosbag2 directory must contain metadata.yaml",
            )
        actual = sha256_path_bundle(bag)
        if actual != self.bag_sha256:
            raise AssetContractError(
                "dataset_bag_hash_mismatch",
                f"bag bundle hash mismatch: expected {self.bag_sha256}, got {actual}",
            )
        return bag


@dataclass(frozen=True)
class DerivationRecipe:
    recipe_id: str
    source_dataset_id: str
    source_dataset_sha256: str
    calibration_id: str
    calibration_sha256: str
    platform_profile: str
    platform_profile_sha256: str
    repository_commit: str
    random_seed: int
    raw: Mapping[str, Any]

    @classmethod
    def from_file(cls, path: str | Path) -> "DerivationRecipe":
        return cls.from_dict(load_yaml_mapping(path))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "DerivationRecipe":
        return cls(
            recipe_id=_require_text(data, "recipe_id", code="recipe_id_missing"),
            source_dataset_id=_require_text(
                data, "source_dataset_id", code="recipe_dataset_missing"
            ),
            source_dataset_sha256=_require_hash(
                data.get("source_dataset_sha256"),
                code="recipe_dataset_hash_invalid",
                field="source_dataset_sha256",
            ),
            calibration_id=_require_text(
                data, "calibration_id", code="recipe_calibration_missing"
            ),
            calibration_sha256=_require_hash(
                data.get("calibration_sha256"),
                code="recipe_calibration_hash_invalid",
                field="calibration_sha256",
            ),
            platform_profile=_require_text(
                data, "platform_profile", code="recipe_platform_missing"
            ),
            platform_profile_sha256=_require_hash(
                data.get("platform_profile_sha256"),
                code="recipe_platform_hash_invalid",
                field="platform_profile_sha256",
            ),
            repository_commit=_require_text(
                data, "repository_commit", code="recipe_repository_commit_missing"
            ),
            random_seed=int(data.get("random_seed", 0)),
            raw=dict(data),
        )

    def assert_compatible(
        self, dataset: DatasetBinding, dataset_binding_sha256: str
    ) -> None:
        if self.source_dataset_id != dataset.dataset_id:
            raise AssetContractError(
                "recipe_dataset_id_mismatch",
                "recipe source_dataset_id differs from dataset binding",
            )
        if self.source_dataset_sha256 != dataset_binding_sha256:
            raise AssetContractError(
                "recipe_dataset_hash_mismatch",
                "recipe source_dataset_sha256 differs from dataset binding content",
            )
        if self.calibration_id != dataset.calibration_id:
            raise AssetContractError(
                "recipe_calibration_id_mismatch",
                "recipe calibration_id differs from dataset binding",
            )
        if self.calibration_sha256 != dataset.calibration_sha256:
            raise AssetContractError(
                "recipe_calibration_hash_mismatch",
                "recipe calibration hash differs from dataset binding",
            )
        if self.platform_profile != dataset.platform_id:
            raise AssetContractError(
                "recipe_platform_id_mismatch",
                "recipe platform_profile differs from dataset binding",
            )
        if self.platform_profile_sha256 != dataset.platform_profile_sha256:
            raise AssetContractError(
                "recipe_platform_hash_mismatch",
                "recipe platform profile hash differs from dataset binding",
            )


@dataclass(frozen=True)
class RoutePolicy:
    policy_id: str
    planning_mode: str
    row_interpretation: str
    use_access_lanes: bool
    use_headland_zones: bool
    minimum_clearance_m: float
    allow_reverse: bool
    unknown_space_allowed: bool
    direction_change_requires_stop: bool
    path_resolution_m: float
    footprint_check_resolution_m: float
    raw: Mapping[str, Any]

    @classmethod
    def from_file(cls, path: str | Path) -> "RoutePolicy":
        return cls.from_dict(load_yaml_mapping(path))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RoutePolicy":
        source = data.get("source")
        constraints = data.get("constraints")
        sampling = data.get("sampling")
        if not isinstance(source, Mapping):
            raise AssetContractError("route_policy_source_missing", "policy source mapping is required")
        if not isinstance(constraints, Mapping):
            raise AssetContractError(
                "route_policy_constraints_missing", "policy constraints mapping is required"
            )
        if not isinstance(sampling, Mapping):
            raise AssetContractError(
                "route_policy_sampling_missing", "policy sampling mapping is required"
            )
        row_interpretation = str(source.get("row_interpretation", "direct_swaths"))
        if row_interpretation not in _ALLOWED_ROW_INTERPRETATIONS:
            raise AssetContractError(
                "route_policy_row_interpretation_invalid",
                f"row_interpretation must be one of {sorted(_ALLOWED_ROW_INTERPRETATIONS)}",
            )
        minimum_clearance = float(constraints.get("minimum_clearance_m", 0.0))
        path_resolution = float(sampling.get("path_resolution_m", 0.05))
        footprint_resolution = float(
            sampling.get("footprint_check_resolution_m", path_resolution)
        )
        for name, value, allow_zero in (
            ("minimum_clearance_m", minimum_clearance, True),
            ("path_resolution_m", path_resolution, False),
            ("footprint_check_resolution_m", footprint_resolution, False),
        ):
            if not math.isfinite(value) or value < 0.0 or (not allow_zero and value <= 0.0):
                raise AssetContractError(
                    "route_policy_numeric_invalid", f"{name} has an invalid value"
                )
        return cls(
            policy_id=_require_text(data, "policy_id", code="route_policy_id_missing"),
            planning_mode=str(source.get("planning_mode", "annotated_rows")),
            row_interpretation=row_interpretation,
            use_access_lanes=bool(source.get("use_access_lanes", True)),
            use_headland_zones=bool(source.get("use_headland_zones", True)),
            minimum_clearance_m=minimum_clearance,
            allow_reverse=bool(constraints.get("allow_reverse", False)),
            unknown_space_allowed=bool(constraints.get("unknown_space_allowed", False)),
            direction_change_requires_stop=bool(
                constraints.get("direction_change_requires_stop", True)
            ),
            path_resolution_m=path_resolution,
            footprint_check_resolution_m=footprint_resolution,
            raw=dict(data),
        )
