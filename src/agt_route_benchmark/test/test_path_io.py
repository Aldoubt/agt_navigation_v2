from pathlib import Path
from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.path_io import read_path_csv, write_path_csv, write_path_geojson


def test_csv_schema_and_round_trip(tmp_path: Path):
    src = [PathPoint(1.0, 2.0, 0.5, "F", "SWATH", "row_01")]
    out = tmp_path / "path.csv"
    write_path_csv(src, out)
    assert out.read_text(encoding="utf-8").splitlines()[0] == "index,x_m,y_m,yaw_rad,direction,segment_type,semantic_ref"
    assert read_path_csv(out) == src


def test_geojson_contains_linestring(tmp_path: Path):
    pts = [
        PathPoint(0, 0, 0, "F", "P2P", ""),
        PathPoint(1, 0, 0, "F", "P2P", ""),
    ]
    out = tmp_path / "path.geojson"
    write_path_geojson(pts, out)
    text = out.read_text(encoding="utf-8")
    assert '"LineString"' in text
    assert '"direction": "F"' in text
