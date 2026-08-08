"""Export route and footprint evidence as GeoJSON for Qt/Web/RViz-side inspection."""

import json
import math
from pathlib import Path

from agt_ui_bridge.platform_profile import load_platform_profile

from .contracts import load_yaml_mapping
from .route_asset import load_route_csv


def write_route_preview(
    route_dir: str | Path,
    *,
    platform_profile_path: str | Path,
    feasibility_result=None,
    maximum_footprints: int = 250,
    extra_invalid_samples=None,
) -> Path:
    route_dir = Path(route_dir).expanduser().resolve()
    samples = load_route_csv(route_dir / "route.csv")
    route_manifest = load_yaml_mapping(route_dir / "route.yaml")
    platform = load_platform_profile(platform_profile_path)
    footprint = tuple(tuple(point) for point in platform["footprint"])

    features = []
    grouped = []
    current = []
    current_id = None
    for sample in samples:
        if sample.segment_id != current_id:
            if current:
                grouped.append(current)
            current = []
            current_id = sample.segment_id
        current.append(sample)
    if current:
        grouped.append(current)

    for group in grouped:
        features.append({
            "type": "Feature",
            "id": group[0].segment_id,
            "properties": {
                "layer": "route_segment",
                "segment_id": group[0].segment_id,
                "direction": group[0].direction,
                "semantic_ref": group[0].semantic_ref,
            },
            "geometry": {
                "type": "LineString",
                "coordinates": [[sample.x, sample.y] for sample in group],
            },
        })

    stride = max(1, int(math.ceil(len(samples) / max(1, maximum_footprints))))
    for sample in samples[::stride]:
        polygon = _transform_footprint(footprint, sample.x, sample.y, sample.yaw)
        features.append({
            "type": "Feature",
            "properties": {
                "layer": "vehicle_footprint",
                "seq": sample.seq,
                "segment_id": sample.segment_id,
                "direction": sample.direction,
            },
            "geometry": {"type": "Polygon", "coordinates": [polygon + [polygon[0]]]},
        })
        if sample.event_ref:
            features.append({
                "type": "Feature",
                "properties": {
                    "layer": "event_anchor",
                    "event_ref": sample.event_ref,
                    "seq": sample.seq,
                },
                "geometry": {"type": "Point", "coordinates": [sample.x, sample.y]},
            })

    seen = set()
    if feasibility_result is not None:
        for item in feasibility_result.geometry_result.invalid_samples[:maximum_footprints]:
            key = _sample_key(item)
            seen.add(key)
            features.append(_invalid_feature(footprint, item, "occupancy_or_kinematics"))
    for item in list(extra_invalid_samples or [])[:maximum_footprints]:
        key = _sample_key(item)
        if key in seen:
            continue
        seen.add(key)
        features.append(_invalid_feature(footprint, item, "semantic_free_space"))

    document = {
        "type": "FeatureCollection",
        "schema_version": 1,
        "frame_id": "map",
        "properties": {
            "route_id": str(route_manifest.get("route_id", "")),
            "revision": int(route_manifest.get("revision", 0)),
            "feasibility_status": (
                feasibility_result.report["status"] if feasibility_result is not None else "NOT_EVALUATED"
            ),
        },
        "features": features,
    }
    output = route_dir / "preview.geojson"
    output.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return output


def _invalid_feature(footprint, item, reason):
    polygon = _transform_footprint(footprint, item.pose.x, item.pose.y, item.pose.yaw)
    return {
        "type": "Feature",
        "properties": {
            "layer": "invalid_footprint",
            "segment_index": int(item.segment_index),
            "reason": reason,
        },
        "geometry": {"type": "Polygon", "coordinates": [polygon + [polygon[0]]]},
    }


def _sample_key(item):
    return (
        int(item.segment_index),
        round(float(item.pose.x), 6),
        round(float(item.pose.y), 6),
        round(float(item.pose.yaw), 6),
    )


def _transform_footprint(footprint, x: float, y: float, yaw: float):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        [x + cosine * px - sine * py, y + sine * px + cosine * py]
        for px, py in footprint
    ]
