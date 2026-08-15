import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication, QGraphicsScene

from agt_offline_assets.route_debug_dataset import RouteDebugDataset
from agt_map_workbench.route_debug_view import RouteDebugSceneController


def _qapp():
    return QApplication.instance() or QApplication([])


def _dataset() -> RouteDebugDataset:
    return RouteDebugDataset(
        run_dir=Path("."),
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
    return {
        "type": "FeatureCollection",
        "agt_schema": "agt_route_debug_overlay/v1",
        "frame_id": "map",
        "validation_scope": "DEBUG_RENDER_ONLY",
        "features": [
            {
                "type": "Feature",
                "properties": {
                    "feature_id": "aisle:aisle_001",
                    "layer_group": "structure",
                    "layer_key": "structure.aisles",
                    "feature_kind": "AISLE_CENTERLINE",
                    "frame_id": "map",
                    "status": "ACCEPTED",
                    "source_asset": "aisle_graph.yaml",
                    "source_id": "aisle_001",
                    "source_field": "aisles[].centerline_xyz",
                    "is_failure": False,
                    "inspector": {"STRUCTURE": {"aisle_id": "aisle_001"}},
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[0.0, 1.0], [3.0, 1.0]],
                },
            }
        ],
    }


def test_route_debug_controller_preserves_child_selection_and_world_y_flip():
    app = _qapp()
    scene = QGraphicsScene()
    controller = RouteDebugSceneController(scene)
    selected = []
    controller.featureSelected.connect(selected.append)

    controller.set_content(_dataset(), _overlay())
    controller.set_active(True)
    controller.apply_preset("coverage")

    group = controller._groups["structure.aisles"]
    assert not group.handlesChildEvents()
    assert controller.layer_visible("structure.aisles")
    assert not controller.layer_visible("motion.reverse")
    assert controller.select_feature("aisle:aisle_001")
    app.processEvents()

    item = next(item for item in scene.selectedItems() if item.data(0) == "aisle:aisle_001")
    assert item.sceneBoundingRect().center().y() < 0.0
    assert selected[-1]["source_id"] == "aisle_001"


def test_route_debug_controller_hides_owned_layers_when_inactive():
    _qapp()
    scene = QGraphicsScene()
    controller = RouteDebugSceneController(scene)
    controller.set_content(_dataset(), _overlay())
    controller.set_active(True)
    assert controller._groups["structure.aisles"].isVisible()

    controller.set_active(False)
    assert not controller._groups["structure.aisles"].isVisible()
