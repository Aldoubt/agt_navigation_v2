# V2.5 Component Status

This is the current component-oriented status view. Historical implementation
order and migration evidence are kept under [`docs/archive/`](../archive/).
Statuses use `IMPLEMENTED`, `SYSTEM-INTEGRATED`, `VEHICLE-PENDING`, `RESERVED`,
and `OPTIONAL`; they do not imply vehicle acceptance unless explicitly stated.

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
| ROUTE navigation core | RESERVED | V25-09 target; no runtime implementation in V25-08. |
| Sparse correction / anchor recovery | RESERVED | V25-10 target; candidates do not own `map -> odom`. |
| Rolling local map / ground factors | RESERVED | V25-11 and P1 state-estimation work after the first mission. |
| GNSS, wheel factor, GTSAM/iSAM2 | RESERVED | P1 state-estimation robustness track. |
| Optional ESDF / advanced local planning | OPTIONAL | V25-12 derived representation; not a default prerequisite. |
| STD / Scan Context and long-term map | RESERVED | P2 long-term agricultural navigation research. |
| Qt/Web operator tooling | OPTIONAL | Clients remain outside business state ownership and motion boundaries. |
