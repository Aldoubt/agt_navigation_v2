from __future__ import annotations
import argparse, json
from .frame_verification import build_frame_alignment_report
from .prepare import prepare_project, SourceIdentityError
from .status import build_project_status

def main(argv=None):
    parser = argparse.ArgumentParser(prog="agt-map"); sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare"); p.add_argument("pcd"); p.add_argument("--preset", required=True); p.add_argument("--output", required=True); p.add_argument("--frame-id", default="map"); p.add_argument("--site-id"); p.add_argument("--profile"); p.add_argument("--resume", action="store_true"); p.add_argument("--alignment"); p.add_argument("--canonical-map-yaml"); p.add_argument("--v25-map-revision")
    s = sub.add_parser("status"); s.add_argument("project"); s.add_argument("--json", action="store_true")
    v = sub.add_parser("verify-frame"); v.add_argument("project"); v.add_argument("--route-csv"); v.add_argument("--semantic-map")
    args = parser.parse_args(argv)
    if args.command == "status":
        try: payload = build_project_status(args.project)
        except (ValueError, OSError): return 4
        if args.json: print(json.dumps(payload, sort_keys=True))
        else: print(f"Project : {args.project}\nState   : {payload['project_state']}\nPreset  : {payload['preset']}\nNext    : {payload['next_action']['command']}")
        return 0
    if args.command == "verify-frame":
        try: report = build_frame_alignment_report(args.project, route_csv=args.route_csv, semantic_map=args.semantic_map)
        except (ValueError, OSError): return 4
        print(json.dumps(report, sort_keys=True))
        return 0 if report["status"] == "PASS" else 3
    try:
        result = prepare_project(
            args.pcd,
            preset_name=args.preset,
            output_dir=args.output,
            declared_frame_id=args.frame_id,
            site_id=args.site_id,
            profile_path=args.profile,
            resume=args.resume,
            alignment_path=args.alignment,
            canonical_map_yaml=args.canonical_map_yaml,
            v25_map_revision=args.v25_map_revision,
        )
        print(f"Project : {result.project_dir}\nState   : {result.project_state}\nNext    : agt-map review {result.project_dir}")
        return 2 if result.human_required else 0
    except (FileNotFoundError, ValueError, SourceIdentityError): return 3 if not args.resume else 4
    except Exception: return 5
