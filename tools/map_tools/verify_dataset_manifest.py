#!/usr/bin/env python3
"""Verify immutable artifact identities recorded in a runtime dataset manifest."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_artifact_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def iter_hashed_artifacts(manifest: dict):
    for name, record in (manifest.get("artifacts") or {}).items():
        yield name, record
    alignment = manifest.get("alignment_candidate") or {}
    for name in ("record", "matrix"):
        if alignment.get(name):
            yield f"alignment_{name}", alignment[name]


def verify_manifest(manifest_path: Path) -> tuple[list[str], list[str]]:
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    errors = []
    results = []
    if manifest.get("schema") != "agt_navigation_dataset_manifest/v1":
        errors.append(f"unsupported schema: {manifest.get('schema')!r}")
    for name, record in iter_hashed_artifacts(manifest):
        path_value = record.get("path")
        expected = record.get("sha256")
        if not path_value or not expected:
            errors.append(f"{name}: path or sha256 missing")
            continue
        path = resolve_artifact_path(path_value)
        if not path.is_file():
            errors.append(f"{name}: missing {path}")
            continue
        actual = sha256_file(path)
        if actual != expected:
            errors.append(f"{name}: SHA256 mismatch ({path})")
        else:
            results.append(f"PASS {name}: {path_value}")
    return results, errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path, help="Dataset manifest YAML")
    parser.add_argument(
        "--require-navigation-ready",
        action="store_true",
        help="Also fail unless navigation_closed_loop_ready is true",
    )
    args = parser.parse_args()
    manifest_path = args.manifest.expanduser().resolve()
    if not manifest_path.is_file():
        print(f"ERROR manifest missing: {manifest_path}", file=sys.stderr)
        return 2
    results, errors = verify_manifest(manifest_path)
    for result in results:
        print(result)
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    readiness = manifest.get("readiness") or {}
    print(f"STATE {manifest.get('state', 'unknown')}")
    print(f"NAVIGATION_READY {bool(readiness.get('navigation_closed_loop_ready', False))}")
    if args.require_navigation_ready and not readiness.get("navigation_closed_loop_ready", False):
        errors.append(f"navigation blocked: {readiness.get('blocking_reason', 'unspecified')}")
    for error in errors:
        print(f"ERROR {error}", file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
