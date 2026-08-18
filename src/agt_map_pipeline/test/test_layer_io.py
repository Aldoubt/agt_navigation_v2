import numpy as np
from agt_offline_assets.navigation_map_derivation import NavigationMapResult, GroundRelativeNavigationConfig
from agt_map_pipeline.traversability_preview import derive_unbounded_traversability_preview

def _nav():
    occ = np.array([[254, 0, 205]], dtype=np.uint8)
    return NavigationMapResult(0.1, 0.0, 0.0, 3, 1, np.zeros((1,3)), np.ones((1,3), bool), np.ones((1,3), np.int32), np.ones((1,3), np.int32), np.array([[0,2,0]], np.int32), np.zeros((1,3)), np.zeros((1,3)), occ, GroundRelativeNavigationConfig())

def test_unbounded_preview_never_recovers_unknown():
    nav = _nav()
    corridor = type("Corridor", (), {"aisle_geometric_envelope": np.array([[True, False, True]])})()
    preview = derive_unbounded_traversability_preview(nav, corridor)
    assert preview["status"] == "CANDIDATE_UNBOUNDED"
    assert np.array_equal(preview["observed_free"], nav.occupancy == 254)
    assert np.array_equal(preview["sensor_obstacle"], nav.occupancy == 0)
    assert np.array_equal(preview["unknown"], nav.occupancy == 205)
    assert "inferred_traversable" not in preview
