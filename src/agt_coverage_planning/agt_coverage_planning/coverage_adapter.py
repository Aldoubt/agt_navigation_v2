"""Convert a validated semantic task into an OpenNav coverage request spec."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from agt_ui_bridge.map_transform import MapGeometry
from agt_ui_bridge.coverage_preview import (
    CoveragePreviewError,
    derive_inter_row_aisles,
    explicit_access_lane_swaths,
)
from agt_ui_bridge.semantic_validation import ValidationContext, validate_task


FLOAT_TOLERANCE = 1e-6
ROW_GML_MIN_POINT_COUNT = 3
GML_NAMESPACE = "http://www.opengis.net/gml"
ET.register_namespace("gml", GML_NAMESPACE)


class CoverageAdapterError(ValueError):
    """Stable, operator-facing failure raised before an action goal is sent."""

    def __init__(self, code, message, object_id="<document>"):
        super().__init__(message)
        self.code = str(code)
        self.object_id = str(object_id)


@dataclass(frozen=True)
class CoverageRequestSpec:
    planning_mode: str
    frame_id: str
    polygons: tuple
    gml_text: str
    swath_angle: float
    robot_width: float
    operation_width: float
    min_turning_radius: float
    headland_width: float
    allow_reverse: bool
    generate_headland: bool
    swath_objective: str
    swath_mode: str
    row_swath_mode: str
    route_mode: str
    path_mode: str
    path_continuity_mode: str
    requested_swaths: tuple


def prepare_coverage_request(task, platform):
    """Validate and convert a LoadedSemanticTask without importing ROS messages."""
    _validate_profile_snapshot(task, platform)
    try:
        map_geometry = MapGeometry.from_nav2_yaml(task.base_map_path)
    except (KeyError, OSError, TypeError, ValueError) as exc:
        raise CoverageAdapterError("base_map_invalid", str(exc)) from exc

    context = ValidationContext(
        map_geometry=map_geometry,
        navigation_footprint=tuple(tuple(point) for point in platform["footprint"]),
        base_map_path=task.base_map_path,
    )
    report = validate_task(task.semantic_map, task.coverage, context=context)
    errors = [issue for issue in report.issues if issue.severity == "ERROR"]
    if errors:
        issue = errors[0]
        raise CoverageAdapterError(issue.code, issue.message, issue.object_id)
    if task.read_only:
        raise CoverageAdapterError(
            "semantic_task_read_only",
            "semantic task is read-only and cannot be planned",
        )

    enabled = [feature for feature in task.semantic_map.features if feature.enabled]
    fields = [feature for feature in enabled if feature.feature_type == "field_boundary"]
    exclusions = [
        feature for feature in enabled if feature.feature_type == "exclusion_zone"
    ]
    directions = [
        feature for feature in enabled if feature.feature_type == "work_direction"
    ]
    rows = [feature for feature in enabled if feature.feature_type == "row_centerline"]

    if len(fields) != 1:
        raise CoverageAdapterError(
            "unsupported_field_count",
            "TASK-09 supports exactly one enabled field_boundary per request",
        )
    for feature in fields + exclusions:
        if len(feature.coordinates) != 1:
            raise CoverageAdapterError(
                "nested_polygon_rings_unsupported",
                "use exclusion_zone features instead of nested polygon rings",
                feature.id,
            )

    direction = directions[0]
    start, end = direction.coordinates[0], direction.coordinates[-1]
    swath_angle = math.atan2(end[1] - start[1], end[0] - start[0]) % math.pi
    coverage = task.coverage
    polygons = tuple(
        tuple((float(point[0]), float(point[1])) for point in feature.coordinates[0])
        for feature in fields + exclusions
    )

    if coverage.planning_mode == "annotated_rows":
        if coverage.row_interpretation == "crop_centerlines":
            try:
                derived_map = derive_inter_row_aisles(task.semantic_map)
            except CoveragePreviewError as exc:
                raise CoverageAdapterError(
                    "inter_row_aisle_derivation_failed", str(exc), "row_centerline"
                ) from exc
            rows = [
                feature
                for feature in derived_map.features
                if feature.enabled and feature.feature_type == "row_centerline"
            ]
        else:
            rows += explicit_access_lane_swaths(task.semantic_map)
        if len(rows) < 2:
            raise CoverageAdapterError(
                "insufficient_annotated_rows",
                "annotated_rows mode requires at least two enabled row_centerline features",
                "row_centerline",
            )
        gml_text = _build_rows_gml(fields[0], exclusions, rows)
        polygons = ()
        generate_headland = False
        swath_mode = "UNKNOWN"
        row_swath_mode = "ROWSARESWATHS"
        requested_swaths = tuple(
            (
                (float(row.coordinates[0][0]), float(row.coordinates[0][1])),
                (float(row.coordinates[-1][0]), float(row.coordinates[-1][1])),
            )
            for row in rows
        )
    else:
        gml_text = ""
        generate_headland = coverage.headland_width > 0.0
        swath_mode = "SET_ANGLE"
        row_swath_mode = "UNKNOWN"
        requested_swaths = ()

    return CoverageRequestSpec(
        planning_mode=coverage.planning_mode,
        frame_id="map",
        polygons=polygons,
        gml_text=gml_text,
        swath_angle=swath_angle,
        robot_width=float(platform["robot_width"]),
        operation_width=float(coverage.operation_width),
        min_turning_radius=float(platform["min_turning_radius"]),
        headland_width=float(coverage.headland_width),
        allow_reverse=bool(coverage.allow_reverse),
        generate_headland=generate_headland,
        swath_objective="LENGTH" if coverage.planning_mode == "polygon" else "UNKNOWN",
        swath_mode=swath_mode,
        row_swath_mode=row_swath_mode,
        route_mode="BOUSTROPHEDON",
        path_mode="REEDS_SHEPP" if coverage.allow_reverse else "DUBIN",
        path_continuity_mode="CONTINUOUS",
        requested_swaths=requested_swaths,
    )


def _validate_profile_snapshot(task, platform):
    coverage = task.coverage
    if platform["name"] != coverage.robot_profile:
        raise CoverageAdapterError(
            "robot_profile_mismatch",
            "coverage robot_profile differs from the selected platform profile",
        )
    for field_name in ("robot_width", "min_turning_radius"):
        expected = float(platform[field_name])
        actual = float(getattr(coverage, field_name))
        if not math.isclose(expected, actual, abs_tol=FLOAT_TOLERANCE):
            raise CoverageAdapterError(
                f"{field_name}_profile_mismatch",
                f"coverage {field_name} differs from the canonical platform profile",
            )


def _build_rows_gml(field, exclusions, rows):
    root = ET.Element("GAOS_parcel", {"id": "agt_semantic_coverage"})
    field_element = ET.SubElement(root, "Field", {"id": field.id})
    geometry = ET.SubElement(field_element, "geometry")
    polygon = ET.SubElement(
        geometry, f"{{{GML_NAMESPACE}}}Polygon", {"srsName": "map"}
    )
    _append_ring(polygon, "outerBoundaryIs", field.coordinates[0])
    for exclusion in exclusions:
        _append_ring(polygon, "innerBoundaryIs", exclusion.coordinates[0])

    normalized_coordinates = _equalize_row_point_counts(rows)
    for row_id, (row, row_coordinates) in enumerate(
        zip(rows, normalized_coordinates), start=1
    ):
        row_element = ET.SubElement(root, "Row", {"id": str(row_id)})
        geometry = ET.SubElement(row_element, "geometry")
        line = ET.SubElement(
            geometry, f"{{{GML_NAMESPACE}}}LineString", {"srsName": "map"}
        )
        coordinates = ET.SubElement(line, f"{{{GML_NAMESPACE}}}coordinates")
        coordinates.text = _coordinate_text(row_coordinates)
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def _equalize_row_point_counts(rows):
    """Equalize rows and avoid OpenNav's degenerate two-point swath output."""
    point_count = max(
        ROW_GML_MIN_POINT_COUNT,
        max(len(row.coordinates) for row in rows),
    )
    return [_resample_line(row.coordinates, point_count) for row in rows]


def _line_length(points):
    return sum(math.dist(first, second) for first, second in zip(points, points[1:]))


def _resample_line(points, point_count):
    if len(points) == point_count:
        return points

    segment_lengths = [
        math.dist(first, second) for first, second in zip(points, points[1:])
    ]
    total_length = sum(segment_lengths)
    if total_length <= FLOAT_TOLERANCE:
        return [points[0]] * point_count

    targets = [total_length * index / (point_count - 1) for index in range(point_count)]
    output = []
    segment_index = 0
    traversed = 0.0
    for target in targets:
        while (
            segment_index < len(segment_lengths) - 1
            and traversed + segment_lengths[segment_index] < target
        ):
            traversed += segment_lengths[segment_index]
            segment_index += 1
        segment_length = segment_lengths[segment_index]
        ratio = 0.0 if segment_length <= FLOAT_TOLERANCE else (
            target - traversed
        ) / segment_length
        start = points[segment_index]
        end = points[segment_index + 1]
        output.append(
            [
                float(start[0]) + (float(end[0]) - float(start[0])) * ratio,
                float(start[1]) + (float(end[1]) - float(start[1])) * ratio,
            ]
        )
    return output


def _append_ring(polygon, boundary_name, ring):
    boundary = ET.SubElement(polygon, f"{{{GML_NAMESPACE}}}{boundary_name}")
    linear_ring = ET.SubElement(boundary, f"{{{GML_NAMESPACE}}}LinearRing")
    coordinates = ET.SubElement(linear_ring, f"{{{GML_NAMESPACE}}}coordinates")
    coordinates.text = _coordinate_text(ring)


def _coordinate_text(points):
    return " ".join(f"{float(point[0]):.12g},{float(point[1]):.12g}" for point in points)
