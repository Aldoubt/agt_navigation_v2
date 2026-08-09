# Map & Route Studio Workflow

1. Open a READY map manifest in the existing Qt5 Studio.
2. Clone it to a new map version. The clone is DRAFT and records the parent
   `map_content_sha256`.
3. Edit occupancy pixels or semantic features. READY assets remain read-only.
4. Save raster edits through `map_edit_record.yaml` with operation, mode,
   parameters and resulting image hash.
5. Run Validate Map. Identity, asset hashes, navigation products and semantic
   validity must pass before promotion.
6. Generate a DRAFT Route using the selected policy and vehicle profile. The
   policy selects `connector_backend: straight`; unknown values fail closed.
7. Run Validate Route. The existing full-footprint, clearance, kinematic,
   semantic and unknown/out-of-bounds checks produce feasibility and preview
   evidence. A valid route bound to a READY map can be promoted READY.
8. Review lane, connector, direction, segment, footprint and invalid-sample
   layers. Preview is advisory and cannot start control.
9. The existing runtime consumes only immutable READY Route revisions through
   `ExecuteWaypointTask`; no new public Route Action is created.

Map and Route edits create new revisions rather than mutating accepted assets.
