#!/usr/bin/env python3

import argparse
from pathlib import Path
import sys

from agt_map_manager.registry import MapRegistry


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Manage immutable AGT map versions")
    parser.add_argument("--root", required=True, help="runtime/maps directory")
    subparsers = parser.add_subparsers(dest="command", required=True)
    register = subparsers.add_parser("register")
    register.add_argument("manifest", type=Path)
    import_parser = subparsers.add_parser("import")
    import_parser.add_argument("--map-id", required=True)
    import_parser.add_argument("--map-yaml", required=True, type=Path)
    import_parser.add_argument("--pcd", required=True, type=Path)
    import_parser.add_argument("--processing-record", required=True, type=Path)
    import_parser.add_argument("--platform-profile", default="")
    list_parser = subparsers.add_parser("list")
    list_parser.add_argument("--map-id")
    list_parser.add_argument("--state")
    validate = subparsers.add_parser("validate")
    validate.add_argument("version_id")
    activate = subparsers.add_parser("activate")
    activate.add_argument("version_id")
    pin = subparsers.add_parser("pin")
    pin.add_argument("version_id")
    unpin = subparsers.add_parser("unpin")
    unpin.add_argument("version_id")
    archive = subparsers.add_parser("archive")
    archive.add_argument("version_id")
    delete = subparsers.add_parser("delete")
    delete.add_argument("version_id")
    purge = subparsers.add_parser("purge")
    purge.add_argument("version_id")
    rebuild = subparsers.add_parser("rebuild")
    del rebuild
    args = parser.parse_args(argv)
    registry = MapRegistry(args.root)
    if args.command == "register":
        result = registry.register_manifest(args.manifest)
        print(result)
        return 0 if result.valid else 2
    if args.command == "import":
        result = registry.import_legacy(map_id=args.map_id, map_yaml=args.map_yaml, localization_pcd=args.pcd, processing_record=args.processing_record, platform_profile=args.platform_profile)
        print(result)
        return 0 if result.valid else 2
    if args.command == "list":
        for row in registry.list_versions(map_id=args.map_id, state=args.state):
            print(row)
        return 0
    if args.command == "validate":
        row = registry._row(args.version_id)
        result = registry.validate_manifest(row["manifest_path"])
        print(result)
        return 0 if result.valid else 2
    if args.command == "activate":
        result = registry.activate(args.version_id)
        print(result)
        return 0 if result.valid else 2
    if args.command == "pin":
        registry.set_pinned(args.version_id, True)
        return 0
    if args.command == "unpin":
        registry.set_pinned(args.version_id, False)
        return 0
    if args.command == "archive":
        registry.archive(args.version_id)
        return 0
    if args.command == "delete":
        print(registry.soft_delete(args.version_id))
        return 0
    if args.command == "purge":
        registry.purge(args.version_id)
        return 0
    print(f"rebuilt {registry.rebuild_index()} versions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
