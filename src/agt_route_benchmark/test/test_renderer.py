from pathlib import Path
from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.renderer import render_route


def test_renderer_writes_svg_png_and_pdf(tmp_path: Path):
    points = [
        PathPoint(0, 0, 0, "F", "SWATH", "row_1"),
        PathPoint(1, 0, 0, "F", "CONNECTION", "headland_A"),
        PathPoint(1, 1, 1.57, "F", "SWATH", "row_2"),
    ]
    outputs = render_route(points, tmp_path / "figure", title="S06 / ours", map_extent=(-1, 2, -1, 2))
    assert {p.suffix for p in outputs} == {".svg", ".png", ".pdf"}
    assert all(p.exists() and p.stat().st_size > 0 for p in outputs)
