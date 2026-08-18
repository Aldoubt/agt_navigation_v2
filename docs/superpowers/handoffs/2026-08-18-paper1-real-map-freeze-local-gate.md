# Superseded — Paper I Real-Map Freeze V1 Local Gate

This handoff is superseded by:

`docs/superpowers/handoffs/2026-08-18-paper1-map-first-freeze-local-gate.md`

Reason: the original ordering stopped at missing semantic inputs before exporting and auditing the real occupancy-map revision. That ordering was too strict.

The corrected workflow is **map first**:

```text
processed PCD
  -> generated map
  -> evidence-backed FORCE_FREE / FORCE_OCCUPIED
  -> accepted map
  -> map-only replay QA
  -> human map review
  -> semantic_map.geojson
  -> coverage.yaml
  -> formal curation manifest
  -> three human acceptance gates
  -> site_snapshot.json
```

Missing semantic inputs block the curation manifest and site snapshot, but they do **not** block Workbench map-revision export or map-only QA.

Do not execute this superseded handoff. Use the canonical map-first handoff above.
