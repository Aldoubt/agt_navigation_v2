# V2.5 Component Status

This is the current component-oriented status view. Historical implementation order and migration evidence are kept under [`docs/archive/`](../archive/). Statuses use `IMPLEMENTED`, `SYSTEM-INTEGRATED`, `VEHICLE-PENDING`, `RESERVED`, and `OPTIONAL`; they do not imply vehicle acceptance unless explicitly stated.

| Component | Status | Evidence boundary / next gate |
| --- | --- | --- |
| FAST-LIVO2 mapping adapter | SYSTEM-INTEGRATED | Implemented; bag and offline evidence exist; field acceptance remains separate. |
| Registered cloud and PCD persistence | IMPLEMENTED | Hash-checked persistence and sparse voxel accumulation are implemented. |
| ICP/NDT relocalization | IMPLEMENTED | Backend and quality contracts exist; batch and field acceptance remain. |
| Nav2 waypoint Action | SYSTEM-INTEGRATED | Project Action and safety boundaries are implemented; vehicle metrics remain. |
| Safety and BUNKER chassis | VEHICLE-PENDING | Watchdog and fail-closed boundaries exist; hardware acceptance remains. |
| Semantic map and keepout | IMPLEMENTED | Data and offline products exist; execution remains gated. |
| System health / TaskReadiness | SYSTEM-INTEGRATED | Structured read models and readiness contracts exist. |
| `agt_mission_manager` business boundary | SYSTEM-INTEGRATED | Project ExecuteMission/FSM ownership exists; it remains the single Mission state owner. |
| URDF self-filter geometry | VEHICLE-PENDING | V25-02 code/config/tests implemented; raw/profile/URDF bag comparison and vehicle validation remain. |
| Sensor sync / health | IMPLEMENTED | Stream monitor + system-manager integration; bag/vehicle threshold tuning remains pending. |
| Semantic waypoint mode | IMPLEMENTED | Schema 1.1 validation split, typed waypoint messages/topic and waypoint server mode; Qt/runtime verification remains. |
| BT capability layer / first BT mission | VEHICLE-PENDING | V25-07 full-chain software integration is complete; vehicle validation remains pending. |
| V25-08 architecture & semantics baseline | IMPLEMENTED | MAP/ROUTE/LOCAL, map products, task/route/path semantics, TF authority and optional-ESDF boundaries are frozen without changing runtime ROS interfaces. |
| Calibration / dataset provenance contract | IMPLEMENTED | V25-09A freezes immutable bag, persisted calibration artifact, platform and recipe lineage. |
| Reproducible map derivation / alignment contract | IMPLEMENTED | V25-09A defines site/epoch identity, canonical alignment, cleaning artifacts and quality evidence. |
| Semantic route asset / feasibility contract | IMPLEMENTED | V25-09A defines map/semantic/coverage/vehicle/policy bindings, CSV geometry, tuning revisions and frozen feasibility/preview evidence. |
| Vehicle tracker adapter contract | IMPLEMENTED | V25-09A freezes route-to-controller responsibilities without adding a public Route Action. |
| Offline asset workspace / compliance tooling | IMPLEMENTED | `agt_offline_assets` provides `init-map`, `refresh-map` and read-only `validate-map`; local colcon/pytest validation is the current gate before closure. |
| Semantic Route MVP / preview / feasibility | IMPLEMENTED | `semantic_boustrophedon_mvp` derives annotated-row candidates; existing full-footprint validator + semantic field/exclusion/keepout gate produce feasibility and preview. Straight connectors are candidates only; kinematically invalid routes fail closed. |
| Reverse-aware / kinematic connector planner | RESERVED | Hybrid-A*, State Lattice or Reeds-Shepp connector backend remains the next offline-planner increment; current Route schema already supports F/R. |
| Route tuning revision pipeline | IMPLEMENTED | `lateral_offset` and `speed_scale` create a new DRAFT revision and require revalidation; READY revisions remain immutable. |
| ROUTE navigation core | RESERVED | V25-09B target after READY Route Asset tooling; no runtime implementation yet. |
| Sparse correction / anchor recovery | RESERVED | V25-10 target; correction producers do not own `map -> odom`; one localization TF publisher is selected at runtime. |
| Local environment mapping | RESERVED | V25-11 target; `/agt/map/local_occupancy` is reserved as an `odom`-frame transient rolling product. |
| Ground factor | RESERVED | P1 state-estimation constraint; separate from the local occupancy representation. |
| GNSS, wheel factor, GTSAM/iSAM2 | RESERVED | P1 state-estimation robustness track. |
| Optional ESDF / advanced local planning | OPTIONAL | V25-12 derived representation after local occupancy; not a default prerequisite. |
| STD / Scan Context and long-term map | RESERVED | P2 long-term agricultural navigation research. |
| Qt/Web operator tooling | OPTIONAL | Clients remain outside business state ownership and motion boundaries; route preview UI is offline evidence, not a motion owner. |
