#!/usr/bin/env python3
"""CLI for reproducible offline map, Route Asset, Site Package, and point-cloud preparation."""

import argparse
import json
import sys

from agt_offline_assets import (
    AssetContractError,
    apply_route_tuning,
    create_map_workspace,
    create_route_candidate_asset,
    create_site_package,
    ingest_mapping_session,
    process_pointcloud,
    read_pcd,
    refresh_map_manifest,
    refresh_site_package,
    sha256_path_bundle,
    summarize_pointcloud,
    validate_map_workspace,
    validate_pointcloud_processing,
    validate_route_asset,
    validate_site_package,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="agt_offline_assets")
    sub = parser.add_subparsers(dest="command", required=True)

    hash_path = sub.add_parser("hash-path", help="hash one file or directory bundle deterministically")
    hash_path.add_argument("path")

    inspect_pcd = sub.add_parser(
        "inspect-pcd",
        help="read PCD schema, geometry and optional deterministic field distributions",
    )
    inspect_pcd.add_argument("--input", required=True)
    inspect_pcd.add_argument(
        "--profile",
        action="store_true",
        help="include deterministic sampled scalar-field percentiles",
    )
    inspect_pcd.add_argument(
        "--sample-limit",
        type=int,
        default=200000,
        help="maximum evenly spaced points used for percentile profiling",
    )

    process_pcd = sub.add_parser(
        "process-pointcloud",
        help="execute an immutable reproducible point-cloud processing recipe",
    )
    process_pcd.add_argument("--input", required=True)
    process_pcd.add_argument("--recipe", required=True)
    process_pcd.add_argument("--output-dir", required=True)
    process_pcd.add_argument("--output-name", default="processed.pcd")

    validate_pcd = sub.add_parser(
        "validate-pointcloud-processing",
        help="read-only audit of one point-cloud processing run",
    )
    validate_pcd.add_argument("--run-dir", required=True)

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

    ingest = sub.add_parser(
        "ingest-mapping-session",
        help="freeze finalized MappingSession outputs as pre-alignment PROCESSING evidence",
    )
    ingest.add_argument("--manifest", required=True)
    ingest.add_argument("--session-file", required=True)
    ingest.add_argument("--session-id", default="")
    ingest.add_argument("--candidate-map-yaml", required=True)
    ingest.add_argument("--candidate-map-image", required=True)
    ingest.add_argument("--localization-pcd", required=True)
    ingest.add_argument("--processing-record", required=True)
    ingest.add_argument("--derived-bag", required=True)
    ingest.add_argument("--source-bag")
    ingest.add_argument("--candidate-report")

    refresh = sub.add_parser("refresh-map", help="hash derived products and optionally change map state")
    refresh.add_argument("--manifest", required=True)
    refresh.add_argument("--state", choices=["DRAFT", "PROCESSING", "READY", "INVALID", "ARCHIVED"])

    validate_map = sub.add_parser("validate-map", help="read-only audit of lineage and frozen map assets")
    validate_map.add_argument("--manifest", required=True)

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

    create_site = sub.add_parser(
        "create-site-package",
        help="create a DRAFT Site Package binding one READY map and optional READY routes",
    )
    create_site.add_argument("--sites-root", required=True)
    create_site.add_argument("--map-manifest", required=True)
    create_site.add_argument("--platform-profile", required=True)
    create_site.add_argument("--route-dir", action="append", default=[])
    create_site.add_argument("--site-package-id")

    refresh_site = sub.add_parser(
        "refresh-site-package",
        help="revalidate a DRAFT Site Package and optionally promote it one-way to READY",
    )
    refresh_site.add_argument("--manifest", required=True)
    refresh_site.add_argument("--maps-root", required=True)
    refresh_site.add_argument("--platform-profile", required=True)
    refresh_site.add_argument("--state", choices=["DRAFT", "READY", "INVALID", "ARCHIVED"])

    validate_site = sub.add_parser(
        "validate-site-package",
        help="read-only audit of Site Package identity and bound READY assets",
    )
    validate_site.add_argument("--manifest", required=True)
    validate_site.add_argument("--maps-root", required=True)
    validate_site.add_argument("--platform-profile", required=True)
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "hash-path":
            print(sha256_path_bundle(args.path))
            return 0
        if args.command == "inspect-pcd":
            cloud = read_pcd(args.input)
            summary = summarize_pointcloud(
                cloud,
                include_profile=bool(args.profile),
                sample_limit=int(args.sample_limit),
            )
            payload = {
                "path": args.input,
                "sha256": sha256_path_bundle(args.input),
                **summary,
            }
            print(json.dumps(payload, ensure_ascii=False))
            return 0
        if args.command == "process-pointcloud":
            result = process_pointcloud(
                args.input,
                args.recipe,
                args.output_dir,
                output_name=args.output_name,
            )
            print(json.dumps(result.to_dict(), ensure_ascii=False))
            return 0
        if args.command == "validate-pointcloud-processing":
            result = validate_pointcloud_processing(args.run_dir)
            print(json.dumps(result.to_dict(), ensure_ascii=False))
            return 0 if result.valid else 2
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
        if args.command == "ingest-mapping-session":
            result = ingest_mapping_session(
                args.manifest,
                session_file=args.session_file,
                session_id=args.session_id,
                candidate_map_yaml=args.candidate_map_yaml,
                candidate_map_image=args.candidate_map_image,
                localization_pcd=args.localization_pcd,
                processing_record=args.processing_record,
                derived_bag_directory=args.derived_bag,
                source_bag_path=args.source_bag,
                candidate_report=args.candidate_report,
            )
            print(json.dumps(result.to_dict(), ensure_ascii=False))
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
        if args.command == "validate-map":
            result = validate_map_workspace(args.manifest)
            print(json.dumps(result.to_dict(), ensure_ascii=False))
            return 0 if result.valid else 2
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
        if args.command == "create-site-package":
            manifest_path = create_site_package(
                args.sites_root,
                map_manifest_path=args.map_manifest,
                platform_profile_path=args.platform_profile,
                route_dirs=args.route_dir,
                site_package_id=args.site_package_id,
            )
            print(manifest_path)
            return 0
        if args.command == "refresh-site-package":
            manifest = refresh_site_package(
                args.manifest,
                maps_root=args.maps_root,
                platform_profile_path=args.platform_profile,
                requested_state=args.state,
            )
            print(json.dumps({
                "site_id": manifest.get("site_id"),
                "site_package_id": manifest.get("site_package_id"),
                "state": manifest.get("state"),
                "site_package_content_sha256": manifest.get("site_package_content_sha256"),
                "route_count": len(manifest.get("routes") or []),
            }, ensure_ascii=False))
            return 0
        if args.command == "validate-site-package":
            result = validate_site_package(
                args.manifest,
                maps_root=args.maps_root,
                platform_profile_path=args.platform_profile,
            )
            print(json.dumps(result.to_dict(), ensure_ascii=False))
            return 0 if result.valid else 2
    except (AssetContractError, KeyError, OSError, TypeError, ValueError) as exc:
        code = getattr(exc, "code", "offline_asset_error")
        print(json.dumps({"status": "ERROR", "code": code, "message": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
