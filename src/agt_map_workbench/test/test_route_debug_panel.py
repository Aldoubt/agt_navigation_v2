import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsView

from agt_offline_assets.route_debug_dataset import RouteDebugDataset
from agt_map_workbench import route_debug_panel as panel_module
from agt_map_workbench.route_debug_panel import RouteDebugPanel


def _qapp():
    return QApplication.instance() or QApplication([])


def _dataset(run_dir: Path) -> RouteDebugDataset:
    return RouteDebugDataset(
        run_dir=run_dir.resolve(),
        frame_id="map",
        navigation=None,
        aisle_graph=None,
        turn_zones=None,
        vehicle_profile=None,
        no_go_regions=(),
        aisles=(),
        connector_requests=(),
        connectors=(),
        occupancy_source_masks=None,
        asset_states=(),
    )


def _overlay():
    def feature(feature_id, layer_key, kind, source_id, coordinates):
        return {
            "type": "Feature",
            "properties": {
                "feature_id": feature_id,
                "layer_group": layer_key.split(".", 1)[0],
                "layer_key": layer_key,
                "feature_kind": kind,
                "frame_id": "map",
                "status": "ACCEPTED",
                "source_asset": "fixture.yaml",
                "source_id": source_id,
                "source_field": "fixture",
                "is_failure": False,
                "inspector": {"STRUCTURE": {"aisle_id": source_id}},
            },
            "geometry": {"type": "LineString", "coordinates": coordinates},
        }

    return {
        "type": "FeatureCollection",
        "agt_schema": "agt_route_debug_overlay/v1",
        "frame_id": "map",
        "validation_scope": "DEBUG_RENDER_ONLY",
        "features": [
            feature(
                "aisle:aisle_001",
                "structure.aisles",
                "AISLE_CENTERLINE",
                "aisle_001",
                [[0.0, 1.0], [3.0, 1.0]],
            ),
            feature(
                "coverage:001:aisle_001",
                "coverage.order",
                "COVERAGE_TRAVERSAL",
                "aisle_001",
                [[0.0, 1.0], [3.0, 1.0]],
            ),
        ],
    }


def test_panel_loads_directory_writes_only_debug_overlay_and_populates_inspector(
    monkeypatch, tmp_path: Path
):
    app = _qapp()
    run_dir = tmp_path / "debug_run"
    run_dir.mkdir()
    dataset = _dataset(run_dir)
    overlay = _overlay()
    writes = []

    monkeypatch.setattr(panel_module, "load_route_debug_dataset", lambda path: dataset)
    monkeypatch.setattr(panel_module, "build_route_debug_overlay", lambda loaded: overlay)

    def fake_write(payload, path, *, overwrite=False):
        assert payload is overlay
        assert overwrite is True
        output = Path(path)
        output.write_text("{}\n", encoding="utf-8")
        writes.append(output)
        return output

    monkeypatch.setattr(panel_module, "write_route_debug_overlay", fake_write)

    scene = QGraphicsScene()
    view = QGraphicsView(scene)
    panel = RouteDebugPanel(scene, view)
    panel.set_active(True)
    loaded = panel.load_run_directory(run_dir)
    app.processEvents()

    assert loaded is dataset
    assert writes == [run_dir / "route_debug_overlay.geojson"]
    assert (run_dir / "route_debug_overlay.geojson").is_file()
    assert panel.current_run_directory() == run_dir.resolve()
    assert panel.controller.select_feature("aisle:aisle_001")
    app.processEvents()
    assert "aisle_001" in panel.inspector_text()
    assert "STRUCTURE" in panel.inspector_text()


def test_panel_presets_keep_missing_reverse_layer_disabled(monkeypatch, tmp_path: Path):
    app = _qapp()
    run_dir = tmp_path / "debug_run"
    run_dir.mkdir()
    dataset = _dataset(run_dir)
    overlay = _overlay()

    monkeypatch.setattr(panel_module, "load_route_debug_dataset", lambda path: dataset)
    monkeypatch.setattr(panel_module, "build_route_debug_overlay", lambda loaded: overlay)
    monkeypatch.setattr(
        panel_module,
        "write_route_debug_overlay",
        lambda payload, path, *, overwrite=False: Path(path),
    )

    scene = QGraphicsScene()
    view = QGraphicsView(scene)
    panel = RouteDebugPanel(scene, view)
    panel.set_active(True)
    panel.load_run_directory(run_dir)
    panel.apply_preset("coverage")
    app.processEvents()

    assert panel.layer_enabled("coverage.order")
    assert not panel.layer_enabled("motion.reverse")
