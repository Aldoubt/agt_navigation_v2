from __future__ import annotations
import numpy as np

def derive_unbounded_traversability_preview(navigation, corridor, *, frame_id="map"):
    occ = np.asarray(navigation.occupancy)
    return {"schema": "agt_traversability_preview/v1", "status": "CANDIDATE_UNBOUNDED", "frame_id": frame_id, "observed_free": occ == 254, "sensor_obstacle": occ == 0, "unknown": occ == 205, "aisle_geometric_envelope": np.asarray(corridor.aisle_geometric_envelope, dtype=bool)}

