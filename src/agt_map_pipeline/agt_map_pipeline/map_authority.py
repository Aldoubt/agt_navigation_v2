from __future__ import annotations

from pathlib import Path
from typing import Mapping

import yaml

from .hashing import file_sha256


MAP_AUTHORITY_SCHEMA = "agt_v25_map_authority_binding/v1"
MAP_AUTHORITY_NAME = "V25_MAP_WORKBENCH"
MAP_AUTHORITY_STATUS = "BOUND_VERIFIED"


class MapAuthorityError(ValueError):
    """Raised when a V25 map revision cannot be trusted as Paper map authority."""


def _require_file(path: Path, label: str) -> Path:
    if not path.is_file():
        raise MapAuthorityError(f"V25 map revision missing {label}: {path}")
    return path.resolve()


def _load_yaml(path: Path, label: str) -> dict:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise MapAuthorityError(f"invalid {label}: {path}") from exc
    if not isinstance(data, dict):
        raise MapAuthorityError(f"invalid {label}: expected mapping")
    return data


def _pgm_dimensions(path: Path) -> tuple[int, int]:
    raw = path.read_bytes()
    tokens: list[bytes] = []
    index = 0
    length = len(raw)
    while len(tokens) < 4 and index < length:
        while index < length and raw[index] in b" \t\r\n":
            index += 1
        if index < length and raw[index:index + 1] == b"#":
            while index < length and raw[index:index + 1] not in {b"\r", b"\n"}:
                index += 1
            continue
        start = index
        while index < length and raw[index] not in b" \t\r\n#":
            index += 1
        if start < index:
            tokens.append(raw[start:index])
    if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
        raise MapAuthorityError(f"unsupported or invalid PGM: {path}")
    try:
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
    except ValueError as exc:
        raise MapAuthorityError(f"invalid PGM header: {path}") from exc
    if width <= 0 or height <= 0 or max_value <= 0:
        raise MapAuthorityError(f"invalid PGM dimensions: {path}")
    return width, height


def _resolve_nav2_map(yaml_path: Path, label: str) -> tuple[Path, dict, dict]:
    data = _load_yaml(yaml_path, label)
    for key in ("image", "resolution", "origin"):
        if key not in data:
            raise MapAuthorityError(f"{label} missing {key}")
    image_path = _require_file(
        (yaml_path.parent / str(data["image"])).resolve(),
        f"{label} image",
    )
    try:
        resolution = float(data["resolution"])
        origin = data["origin"]
        origin_x = float(origin[0])
        origin_y = float(origin[1])
    except (TypeError, ValueError, IndexError) as exc:
        raise MapAuthorityError(f"invalid {label} grid metadata") from exc
    if resolution <= 0.0:
        raise MapAuthorityError(f"invalid {label} resolution")
    width, height = _pgm_dimensions(image_path)
    grid = {
        "resolution_m": resolution,
        "origin_xy_m": [origin_x, origin_y],
        "width": width,
        "height": height,
    }
    return image_path, data, grid


def bind_v25_map_revision(revision_dir: Path | str) -> dict:
    revision = Path(revision_dir).expanduser().resolve()
    if not revision.is_dir():
        raise MapAuthorityError(f"V25 map revision does not exist: {revision}")

    generated_yaml = _require_file(
        revision / "generated" / "navigation_map.yaml", "generated map YAML"
    )
    accepted_yaml = _require_file(
        revision / "accepted" / "navigation_map.yaml", "accepted map YAML"
    )
    derivation = _require_file(revision / "derivation.yaml", "derivation.yaml")

    generated_image, _, generated_grid = _resolve_nav2_map(
        generated_yaml, "generated map YAML"
    )
    accepted_image, _, accepted_grid = _resolve_nav2_map(
        accepted_yaml, "accepted map YAML"
    )
    if generated_grid != accepted_grid:
        raise MapAuthorityError("generated and accepted map grid mismatch")

    derivation_doc = _load_yaml(derivation, "derivation.yaml")
    derivation_frame = str(derivation_doc.get("frame_id", "map"))
    if derivation_frame != "map":
        raise MapAuthorityError("V25 map derivation frame must be map")

    binding = {
        "schema": MAP_AUTHORITY_SCHEMA,
        "authority": MAP_AUTHORITY_NAME,
        "status": MAP_AUTHORITY_STATUS,
        "revision_dir": str(revision),
        "generated_map_yaml_path": str(generated_yaml),
        "generated_map_pgm_path": str(generated_image),
        "accepted_map_yaml_path": str(accepted_yaml),
        "accepted_map_pgm_path": str(accepted_image),
        "derivation_path": str(derivation),
        "generated_map_yaml_sha256": file_sha256(generated_yaml),
        "generated_map_pgm_sha256": file_sha256(generated_image),
        "accepted_map_yaml_sha256": file_sha256(accepted_yaml),
        "accepted_map_pgm_sha256": file_sha256(accepted_image),
        "derivation_sha256": file_sha256(derivation),
        "frame_id": "map",
        "grid": accepted_grid,
    }
    return verify_bound_map_authority(binding)


def validate_map_authority_document(binding: Mapping[str, object]) -> None:
    if binding.get("schema") != MAP_AUTHORITY_SCHEMA:
        raise MapAuthorityError("invalid map authority schema")
    if binding.get("authority") != MAP_AUTHORITY_NAME:
        raise MapAuthorityError("invalid map authority owner")
    if binding.get("status") != MAP_AUTHORITY_STATUS:
        raise MapAuthorityError("invalid map authority status")
    if binding.get("frame_id") != "map":
        raise MapAuthorityError("map authority frame must be map")
    grid = binding.get("grid")
    if not isinstance(grid, Mapping):
        raise MapAuthorityError("map authority grid is missing")
    try:
        resolution = float(grid["resolution_m"])
        origin = grid["origin_xy_m"]
        float(origin[0])
        float(origin[1])
        width = int(grid["width"])
        height = int(grid["height"])
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise MapAuthorityError("invalid map authority grid") from exc
    if resolution <= 0.0 or width <= 0 or height <= 0:
        raise MapAuthorityError("invalid map authority grid")


def verify_bound_map_authority(binding: Mapping[str, object]) -> dict:
    validate_map_authority_document(binding)
    pairs = (
        ("generated map YAML", "generated_map_yaml_path", "generated_map_yaml_sha256"),
        ("generated map PGM", "generated_map_pgm_path", "generated_map_pgm_sha256"),
        ("accepted map YAML", "accepted_map_yaml_path", "accepted_map_yaml_sha256"),
        ("accepted map PGM", "accepted_map_pgm_path", "accepted_map_pgm_sha256"),
        ("derivation", "derivation_path", "derivation_sha256"),
    )
    for label, path_key, hash_key in pairs:
        raw_path = str(binding.get(path_key, ""))
        expected = str(binding.get(hash_key, ""))
        if not raw_path or len(expected) != 64:
            raise MapAuthorityError(f"map authority missing {label} identity")
        path = Path(raw_path)
        if not path.is_file():
            raise MapAuthorityError(f"map authority {label} missing: {path}")
        actual = file_sha256(path)
        if actual != expected:
            raise MapAuthorityError(f"map authority {label} hash mismatch")

    accepted_yaml = Path(str(binding["accepted_map_yaml_path"]))
    accepted_image, _, actual_grid = _resolve_nav2_map(
        accepted_yaml, "accepted map YAML"
    )
    if str(accepted_image) != str(Path(str(binding["accepted_map_pgm_path"])).resolve()):
        raise MapAuthorityError("accepted map YAML image does not match bound PGM")
    expected_grid = dict(binding["grid"])
    if actual_grid != expected_grid:
        raise MapAuthorityError("accepted map grid mismatch")
    return dict(binding)
