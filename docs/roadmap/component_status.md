# V2.5 Component Status

This is the current component-oriented status view. Historical implementation
order and migration evidence are kept under [`docs/archive/`](../archive/).
`DONE` means a stable foundation exists; it does not imply vehicle acceptance.

| Component | Status | Evidence boundary / next gate |
| --- | --- | --- |
| FAST-LIVO2 mapping adapter | DONE | Implemented; bag and offline evidence exist; field acceptance remains separate. |
| Registered cloud and PCD persistence | DONE | Hash-checked persistence and sparse voxel accumulation are implemented. |
| ICP/NDT relocalization | DONE | Backend and quality contracts exist; batch and field acceptance remain. |
| Nav2 waypoint Action | DONE | Project Action and safety boundaries are implemented; vehicle metrics remain. |
| Safety and BUNKER chassis | DONE | Watchdog and fail-closed boundaries exist; hardware acceptance remains. |
| Semantic map and keepout | DONE | Data and offline products exist; execution remains gated. |
| System health / TaskReadiness | DONE | Structured read models and readiness contracts exist. |
| `agt_mission_manager` business boundary | DONE | Project ExecuteMission/FSM ownership exists; it remains the single Mission state owner. |
| URDF self-filter geometry | P0 | V25-02 code/config/tests implemented on feature branch; raw/profile/URDF bag and vehicle validation remain. |
| Sensor sync / health | P0 | IMPLEMENTED: read-only stream health evidence and fail-closed integration; bag/vehicle validation remains PENDING. |
| Semantic waypoint mode | P0 | Add the versioned waypoint extension only after its contract is approved. |
| BT execution engine / first BT mission | P0 | Add BehaviorTree.CPP/Groot2 behind `agt_mission_manager`; compose project Actions, do not add a second mission owner. |
| Rolling local map / ground factors | P1 | Robustness and sensor-factor work after the first mission. |
| GNSS, wheel factor, GTSAM/iSAM2 | P1 | Research-backed localization robustness work. |
| STD / Scan Context and long-term map | P2 | Long-term agricultural navigation research. |
| Qt/Web operator tooling | OPTIONAL | Clients remain outside business state ownership and motion boundaries. |
