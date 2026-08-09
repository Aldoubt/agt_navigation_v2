# V25-09C Map & Route Studio

V25-09C closes the offline authoring loop without introducing a second map,
Route Asset, feasibility validator, public Route Action, or runtime TF owner.

```text
READY Map Revision
  -> clone
DRAFT Map Revision
  -> raster/semantic edit + provenance
  -> map validation
  -> Route generation
  -> existing feasibility + preview
  -> READY Route revision
```

## Revision Semantics

READY map and Route revisions are immutable. Studio edits always target a DRAFT
revision. `clone_map_revision` copies the reproducibility inputs, records the
READY parent content hash, and leaves the target DRAFT. Raster operations are
recorded in `map_edit_record.yaml`; the record is separate from the PCD
cleaning contract.

## Route Planner Boundary

Semantic lane extraction, deterministic lane ordering, boustrophedon sequencing,
lane sampling and curvature calculation remain the existing MVP pipeline. The
inter-lane connector is now selected through a `ConnectorPlannerBackend`
registry. V25-09C implements only `straight`; unknown backends fail closed.
The selected connector is recorded in `route.yaml` as:

```yaml
planner:
  backend: semantic_boustrophedon
  connector_backend: straight
```

Route CSV fields and the existing full-footprint/kinematic/semantic feasibility
validator are unchanged. A READY Route remains the only asset eligible for the
existing ROUTE runtime.

## UI Boundary

The existing Qt5 semantic editor is extended with a Map Revision panel,
read-only READY behavior, clone/validation controls, unified layers, and
Generate/Validate Route controls. It is an offline authoring/review client and
does not own active maps, Mission state, localization, safety, TF, or chassis
commands.

Reeds-Shepp, Hybrid-A*, local occupancy, ESDF, GNSS and graph/factor fusion are
explicitly deferred to later stages.
