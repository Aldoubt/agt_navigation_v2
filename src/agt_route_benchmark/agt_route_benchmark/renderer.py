from __future__ import annotations

from pathlib import Path
from typing import Sequence

from .contracts import PathPoint


def render_route(
    points: Sequence[PathPoint],
    output_stem: Path | str,
    *,
    title: str,
    map_extent: tuple[float, float, float, float] | None = None,
    occupancy_image=None,
    image_origin: str = "lower",
) -> tuple[Path, Path, Path]:
    if not points:
        raise ValueError("cannot render empty path")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    stem = Path(output_stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    if occupancy_image is not None:
        if map_extent is None:
            raise ValueError("map_extent is required with occupancy_image")
        if image_origin not in ("lower", "upper"):
            raise ValueError("image_origin must be lower or upper")
        ax.imshow(occupancy_image, origin=image_origin, extent=map_extent, interpolation="nearest", cmap="gray")
    xs = [p.x_m for p in points]
    ys = [p.y_m for p in points]
    ax.plot(xs, ys, linewidth=1.8)
    ax.scatter([xs[0]], [ys[0]], marker="o", label="start")
    ax.scatter([xs[-1]], [ys[-1]], marker="x", label="goal/end")
    for p in points:
        if p.semantic_ref and p.segment_type == "SWATH":
            ax.annotate(p.semantic_ref, (p.x_m, p.y_m), fontsize=7)
    if map_extent is not None:
        ax.set_xlim(map_extent[0], map_extent[1])
        ax.set_ylim(map_extent[2], map_extent[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.grid(True, linewidth=0.4, alpha=0.4)
    ax.legend(loc="best")
    outputs = tuple(stem.with_suffix(suffix) for suffix in (".svg", ".png", ".pdf"))
    for out in outputs:
        fig.savefig(out, bbox_inches="tight", dpi=180)
    plt.close(fig)
    return outputs
