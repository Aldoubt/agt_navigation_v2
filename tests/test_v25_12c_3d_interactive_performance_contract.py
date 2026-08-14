from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REVIEW = ROOT / "src/agt_map_workbench/agt_map_workbench/review_3d.py"


def test_3d_drag_uses_lightweight_preview_and_restores_stationary_density():
    review = REVIEW.read_text(encoding="utf-8")
    for token in (
        "_INTERACTIVE_CLOUD_LIMIT = 20_000",
        "_INTERACTIVE_DENSE_LAYER_LIMIT = 4_000",
        "if self._dragging and points.shape[0] > _INTERACTIVE_CLOUD_LIMIT",
        "if self._dragging and name in _DENSE_LAYERS",
        "轻量交互预览",
        "完整审查采样",
        "拖动时点云≤",
    ):
        assert token in review
    assert "3D 30 万点" in review
