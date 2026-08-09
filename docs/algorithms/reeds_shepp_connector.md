# Reeds-Shepp Connector

## 1. Agricultural Need

Parallel agricultural rows often end at a headland with a different travel
heading. A straight connector preserves simple XY geometry but can violate the
vehicle's turning radius or require an unrealistic instantaneous heading
change. A Reeds-Shepp connector models a car that can drive forward and
reverse.

## 2. Vehicle Model

The state is `(x, y, yaw)` where `yaw` is vehicle body heading. Primitive
curvature is `+1/rho` for `L`, `0` for `S`, and `-1/rho` for `R`. Integration
uses signed distance `ds`, so reverse motion naturally evolves the body heading
through `d_yaw = curvature * ds`.

## 3. Candidate Families

The implementation evaluates the canonical CSC, CCC, CCCC, CCSC, CSCC and
CCSCC families. Reflection and time reversal generate their symmetric
configurations. Zero-length primitives are removed, and candidates are sorted
deterministically by total absolute length, direction changes, primitive count
and lexical primitive signature.

## 4. Sampling and F/R

`path_resolution_m` is the maximum sample spacing. Each sample stores body
heading and `F` or `R`; it is not assigned a heading from the geometric travel
tangent. Endpoints are snapped to the requested start and goal poses. When the
signed primitive direction changes, the route assembler creates a new
homogeneous `RouteSegment` instead of placing an F/R cusp in one segment.

## 5. Boundary with Feasibility

Reeds-Shepp proves only a curvature-bounded local connection. It does not avoid
field exclusions, keepouts, unknown space or dynamic obstacles. The existing
full-footprint and semantic feasibility validator remains authoritative. A
kinematically valid connector may therefore produce an INVALID Route when it
crosses a keepout; obstacle-aware alternative search belongs to a later Hybrid
A* stage.
