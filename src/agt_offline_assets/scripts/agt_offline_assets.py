#!/usr/bin/env python3
"""CLI for reproducible offline map and Route Asset preparation."""

import argparse
import json
import sys

from agt_offline_assets import (
    AssetContractError,
    apply_route_tuning,
    create_map_workspace,
    create_route_candidate_asset,
    refresh_map_manifest,
    validate_route_asset,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="agt_offline_assets")
    sub = parser.add_subparsers(dest="command", required=True)

    init_map = sub.add_parser("init-map", help="create a reproducible PROCESSING map workspace")
    init_map.add_argument("--maps-root", required=True)
    init_map.add_argument("--map-id", required=True)
    init_map.add_argument("--map-version-id")
    init_map.add_argument("--dataset", required=True)
    init_map.add_argument("--recipe", required=True)
    init_map.add_argument("--site-frame", required=True)
    init_map.add_argument("--alignment", required=True)
    init_map.add_argument("--platform-profile", required=True)
    init_map.add_argument("--calibration", required=True)

    refresh = sub.add_parser("refresh-map", help="hash derived products and optionally change map state")
    refresh.add_argument("--manifest", required=True)
    refresh.add_argument("--state", choices=["DRAFT", "PROCESSING", "READY", "INVALID", "ARCHIVED"])

    derive = sub.add_parser("derive-route", help="derive a DRAFT Route Asset from semantic annotations")
    derive.add_argument("--map-manifest", required=True)
    derive.add_argument("--semantic", required=True)
    derive.add_argument("--coverage")
    derive.add_argument("--policy", required=True)
    derive.add_argument("--platform-profile", required=True)
    derive.add_argument("--route-id", required=True)
    derive.add_argument("--revision", required=True, type=int)
    derive.add_argument("--speed", type=float, default=0.3)

    validate = sub.add_parser("validate-route", help="run full feasibility, preview, and final route promotion")
    validate.add_argument("--route-dir", required=True)
    validate.add_argument("--map-manifest", required=True)
    validate.add_argument("--platform-profile", required=True)
    validate.add_argument("--max-preview-footprints", type=int, default=250)

    tune = sub.add_parser("tune-route", help="apply non-destructive tuning into a new route revision")
    tune.add_argument("--route-dir", required=True)
    tune.add_argument("--tuning", required=True)
    tune.add_argument("--new-revision", required=True, type=int)
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "init-map":
            workspace = create_map_workspace(
                args.maps_root,
                map_id=args.map_id,
                map_version_id=args.map_version_id,
                dataset_binding_path=args.dataset,
                recipe_path=args.recipe,
                site_frame_path=args.site_frame,
                alignment_path=args.alignment,
                platform_profile_path=args.platform_profile,
                calibration_path=args.calibration,
            )
            print(workspace.manifest_path)
            return 0
        if args.command == "refresh-map":
            manifest = refresh_map_manifest(args.manifest, requested_state=args.state)
            print(json.dumps({
                "map_id": manifest.get("map_id"),
                "map_version_id": manifest.get("map_version_id"),
                "state": manifest.get("state"),
                "assets": sorted((manifest.get("assets") or {}).keys()),
            }, ensure_ascii=False))
            return 0
        if args.command == "derive-route":
            route_dir = create_route_candidate_asset(
                map_manifest_path=args.map_manifest,
                semantic_path=args.semantic,
                coverage_path=args.coverage,
                policy_path=args.policy,
                platform_profile_path=args.platform_profile,
                route_id=args.route_id,
                revision=args.revision,
                default_speed_mps=args.speed,
            )
            print(route_dir)
            return 0
        if args.command == "validate-route":
            result = validate_route_asset(
                args.route_dir,
                map_manifest_path=args.map_manifest,
                platform_profile_path=args.platform_profile,
                write_outputs=True,
                maximum_preview_footprints=args.max_preview_footprints,
            )
            print(json.dumps(result.report, ensure_ascii=False))
            return 0 if result.passed else 2
        if args.command == "tune-route":
            new_dir = apply_route_tuning(
                args.route_dir, args.tuning, new_revision=args.new_revision
            )
            print(new_dir)
            return 0
    except (AssetContractError, KeyError, OSError, TypeError, ValueError) as exc:
        code = getattr(exc, "code", "offline_asset_error")
        print(json.dumps({"status": "ERROR", "code": code, "message": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
