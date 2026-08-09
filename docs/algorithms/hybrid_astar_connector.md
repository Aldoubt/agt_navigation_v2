# Hybrid A* Connector

Hybrid A* operates on `(x, y, yaw)` states discretized by grid cell, 72 yaw
bins and direction. Its closed key is `(grid_x, grid_y, yaw_bin, direction)`.
Each expansion integrates forward/reverse `L`, `S` and `R` primitives using the
minimum turning radius and checks all intermediate poses against the supplied
occupancy/semantic context.

The combined heuristic is the maximum of a Reeds-Shepp kinematic lower bound
and a 2D obstacle lower bound. A collision-checked Reeds-Shepp analytic
expansion can terminate a search early; a Reeds-Shepp success without a
collision-free sample is rejected and Hybrid A* continues.

`ConnectorPlanningContext` carries the map resolution/origin, occupancy grid,
vehicle footprint, semantic keepouts/field boundaries and unknown-space policy.
World/grid transforms honor non-zero map origin and yaw. The connector returns
the existing `ConnectorSample` sequence, preserving body heading and F/R
direction. Direction cusps are split by the existing Route assembler.

This is offline local connector planning only. It is not an online replanner,
Nav2 planner plugin, State Lattice, RRT, local occupancy mapper or dynamic
obstacle system. Existing final feasibility remains authoritative.
