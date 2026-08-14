# V25-12C 3D Review + Vehicle Corridor

Status: **IMPLEMENTED MVP, LOCAL ACCEPTANCE PENDING**

## Purpose

The existing 2D Workbench remains the authoring authority for polygon editing,
map-frame calibration, Ground-relative navigation derivation, agricultural row
structure, aisle refinement, and manual overrides.

The new 3D Review plane is review-only. It exists to answer questions that a
top-down view cannot answer reliably:

- Is a detected crop row actually located on the intended ground/ridge surface?
- Does an aisle centerline follow the real terrain height rather than a flat Z?
- Does a proposed vehicle-width corridor intrude into crop rows or other rejected
  cells?
- Are high structures visually separate from the local navigation surface?
- Does a future route remain spatially plausible from front/side/isometric views?

## Workbench layout

```text
AGT Map Workbench
├── 2D Edit / Analysis
│   ├── PCD authoring
│   ├── Map Frame Calibration
│   ├── Ground-relative Navigation Map
│   ├── Hybrid Row / Corridor
│   └── Overrides / export
└── 3D Review
    ├── deterministic sampled PCD
    ├── Ground Surface
    ├── Row Centerline
    ├── Row Structural Band
    ├── Refined Aisle
    ├── Aisle Centerline
    └── Vehicle Corridor
```

3D Review does not call `process_pointcloud`, does not modify the source PCD,
does not edit the final OccupancyGrid, and does not introduce ROS graph
ownership.

## Renderer

The MVP intentionally uses NumPy + Qt/QPainter instead of Open3D/VTK/PyVista.
This keeps the existing ROS 2 Humble deployment lightweight.

PCD rendering uses deterministic even-spacing sampling through `np.linspace`.
Available review limits are 60k, 150k, and 300k points. During interactive
rotation, the canvas temporarily limits the drawn cloud to about 60k points and
restores the selected review sample when interaction stops.

Camera controls:

- left drag: orbit
- right/middle drag: pan
- wheel: zoom
- double click: fit
- presets: top / front / side / isometric

## Shared XYZ semantics

Navigation evidence is lifted back into 3D with the same local Ground Surface:

```text
x = origin_x + (column + 0.5) * resolution
y = origin_y + (row    + 0.5) * resolution
z = ground_height(row, column) + review_offset
```

Therefore row and aisle layers do not silently render on `Z=0`.

## Vehicle Corridor review contract

`agt_offline_assets.vehicle_corridor` defines:

```text
required_width
= vehicle_width + 2 * lateral_safety_margin
```

The output keeps three masks separate:

```text
Required Vehicle Envelope
Safe Vehicle Corridor = Required Envelope ∩ Refined Aisle
Conflict Envelope      = Required Envelope - Refined Aisle
```

This remains review evidence. It does not declare a route feasible and does not
mutate the final navigation map. The same width semantics are intended to be
reused by later Route Feasibility validation.

## Current boundary

The MVP does not yet provide:

- perspective rendering / GPU point splats
- 3D polygon editing
- route polyline import
- swept 3D robot height volume
- explicit roof/overhead collision volume
- vehicle profile asset binding
- final navigation fusion policy

These are follow-on capabilities after real-map 3D review acceptance.

## Local acceptance

Use the current processed greenhouse PCD and existing Ground/Row/Corridor
results.

Acceptance requires:

1. 2D editing behavior is unchanged
2. a separate `3D 审查` tab is available
3. 60k/150k/300k sampled PCD review opens without loading the full PCD into the renderer
4. top/front/side/isometric presets work
5. mouse orbit/pan/zoom is responsive enough for inspection
6. Ground, Row Centerline, Row Structural Band, Refined Aisle, and Aisle Centerline align in XYZ
7. changing vehicle width or safety margin updates Vehicle Corridor without regenerating the PCD
8. 3D review does not change Final PGM / OccupancyGrid
9. existing V25-12B/V25-12C regression tests remain PASS
