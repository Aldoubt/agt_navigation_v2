# Waypoint Task Group Schema

AGT Navigation V2 task groups are versioned, UTF-8 JSON documents for ordered
waypoint work. They are operator input, not an execution state machine. The
project Action server remains the only execution authority:
`/agt/navigation/execute_waypoint_task`.

## Storage

Task groups belong to one immutable map version:

```text
runtime/maps/<map_id>/versions/<map_version_id>/tasks/
  task_index.json
  <task_group_id>.json
  <task_group_id>.json.bak.1
```

The task file stores paths relative to the map version where possible. A Qt
frontend may resolve the file to an absolute path only when sending the Action.
The repository writes a flushed temporary file, fsyncs it, and atomically
renames it. Existing files are retained as numbered backups.

## Contract

`schema_version` is `1`, `frame_id` is always `map`, coordinates are metres and
`yaw` is radians normalized to `[-pi, pi)`. Every waypoint has a stable unique
`id`; disabled points remain editable but are not sent to Nav2. At least one
point must be enabled. `loop_count` is finite and bounded by the configured
maximum even when looping is disabled. Unknown JSON fields and implicit type
coercion are rejected by both the Python server loader and the C++ Qt loader.

`map_binding` records the map ID/version, geometry, YAML/image hashes and the
localization PCD hash. Matching geometry and all hashes is `MATCHED`. Equal
geometry with changed content is `CONTENT_CHANGED` and blocks execution until
the operator explicitly rebinds and saves. Any changed frame, resolution,
dimensions, origin or origin yaw is `GEOMETRY_MISMATCH`; the task is read-only
until it is explicitly copied to a new map version.

The offline validator checks schema/model constraints, map bounds, occupied and
unknown cells, sampled segments, repeated points and repeated whole-path
patterns. `unknown_cell_policy` is `reject` by default and may be `warn` or
`allow`. This is a base-raster check only; it does not replace Nav2 planning,
localization, `agt_safety`, or the runtime canonical footprint check.

At runtime a schema-v1 task additionally requires independently observed active
map ID/version, YAML/image hashes, and the localization PCD hash. The server
does not fill missing active identity from the task document. Legacy points
JSON has no binding and retains the compatibility boundary documented for the
Action.

## Example

```json
{
  "schema_version": 1,
  "task_group_id": "greenhouse_a_inspection_v01",
  "name": "温室 A 区巡检",
  "description": "按既定顺序完成行间巡检",
  "created_at": "2026-07-25T02:00:00+00:00",
  "updated_at": "2026-07-25T02:00:00+00:00",
  "revision": 1,
  "frame_id": "map",
  "map_binding": {
    "map_id": "greenhouse_01",
    "map_version_id": "20260725_v03",
    "map_yaml_path": "navigation/map.yaml",
    "map_yaml_sha256": "sha256:<64 lowercase hex>",
    "map_image_sha256": "sha256:<64 lowercase hex>",
    "localization_pcd_sha256": "sha256:<64 lowercase hex>",
    "resolution": 0.05,
    "width": 1200,
    "height": 800,
    "origin": [-2.475, -30.475, 0.0]
  },
  "execution": {"loop": false, "loop_count": 1},
  "points": [
    {"id": "wp_0001", "name": "入口", "x": 1.25, "y": 2.4, "yaw": 0.0, "enabled": true, "note": ""}
  ]
}
```

Legacy Qt JSON containing `points[].theta` remains accepted by the Action
server. Importing it into the task library generates stable IDs, requires an
explicit current map binding, and never overwrites the source file.
