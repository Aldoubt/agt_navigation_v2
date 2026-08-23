# Paper I P1 Baseline Provenance Report

状态：`PRECHECK_WITH_BLOCKERS`

## Scope

本轮只冻结已存在的 PCD/bag identity 并执行软件 gate。没有重新播放 bag、重新建图、创建 V25 revision 或运行 E1/E2/E3。

## Provenance chain

```text
author-confirmed raw bag
  /home/yangxuan/agt_navigation_v2/runtime/rosbag/green-house
    -> FAST-LIVO2 LIO-only (author-confirmed)
    -> source processed output /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/processed.pcd
    -> Paper resource bundle copy
       runtime/maps/greenhouse_01_map_assets_v01/pointcloud/processed.pcd
    -> V25 accepted-map-shaped resource bundle
```

PCD source and bundle copy have identical SHA256. The processing record shows one `crop_polygon` operation and no transform operation. It declares `frame_id: map` and preserves metric coordinates.

The raw bag metadata identifies a sqlite3 bag with IMU and Livox CustomMsg topics. The repository does not contain an explicit manifest directly binding this `green-house` bag to the locked PCD; the correspondence is recorded as author-confirmed, not inferred from filename alone.

## PCD identity

| Item | Value |
|---|---|
| Absolute path | `/home/yangxuan/agt_navigation_v2_paper1/runtime/maps/greenhouse_01_map_assets_v01/pointcloud/processed.pcd` |
| SHA256 | `069abb9e7f89e78fd8529f224f953df19ec1d582c27dc6fe390b613ab7b0b12a` |
| Size | `2,646,843,519` bytes |
| Points | `82,713,852` |
| Fields | `x y z intensity normal_x normal_y normal_z curvature` |
| Encoding | binary PCD v0.7 |
| Mapping | FAST-LIVO2, LIO-only |

## Transform contract

- Manual scale: **NO**
- Non-rigid warp: **NO**
- Scale: **1.0**
- Separate rigid alignment file: **NOT FOUND**
- Additional P1 transform: **NONE**
- Declared source frame: `map`
- Formal `ALREADY_BAKED`/canonical V25 relation: `TBD-EVIDENCE`

The existing processing record and resource bundle agree on the `map` frame label, but this is not sufficient to claim formal V25 binding without an immutable revision and frame/grid verification.

## Raw bag identity

| Item | Value |
|---|---|
| Absolute path | `/home/yangxuan/agt_navigation_v2/runtime/rosbag/green-house` |
| Type | ROS 2 sqlite3 |
| DB3 SHA256 | `33a7613643d561bc655725a4ae56e6bf5d53dc4a538798ee70251241a2193e5e` |
| Metadata SHA256 | `5eef3d33bf8d24a478889219692ecf7703ea49a851d13af37bd90e9abe3b2010` |
| Duration | `622.994416876 s` |
| Messages | `130830` |
| Topics | `/agt/sensors/imu/data`, `/agt/sensors/lidar/custom` |

## Software gate

- `source /opt/ros/humble/setup.bash && PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning python3 -m pytest -q src/agt_map_pipeline/test`: PASS, 34 tests
- `source /opt/ros/humble/setup.bash && MPLBACKEND=Agg PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets:src/agt_ui_bridge python3 -m pytest -q src/agt_route_benchmark/test`: PASS, 128 tests
- `source /opt/ros/humble/setup.bash && PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning python3 -m pytest -q tests/test_v25_12c_map_workbench_contract.py`: PASS, 22 tests
- `source /opt/ros/humble/setup.bash && python3 -m compileall -q src/agt_map_pipeline/agt_map_pipeline src/agt_map_pipeline/scripts src/agt_route_benchmark/agt_route_benchmark src/agt_route_benchmark/scripts`: PASS

Full logs are under `software_gate/`.

## Blocking status

No formal immutable `greenhouse_01` V25 Map Workbench revision was found. The existing `greenhouse_01_map_assets_v01` resource bundle contains generated/accepted navigation assets and derivation metadata, but its manifest is a resource-bundle manifest, not a verified `V25_MAP_WORKBENCH / BOUND_VERIFIED` project binding. Map-only QA, human map review, frame/semantic verification, site snapshot, and `paper1-method-v0.1` therefore remain incomplete.

See `preflight_report.json` and the two source manifests for machine-readable details.
