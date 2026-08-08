# Offline Bag Replay to PROCESSING Map Workspace

本流程是 V25-09A 的第二段：把已经冻结并写入 Dataset binding 的完整 rosbag2 数据，通过现有 MappingSession 重放建图，并把结果冻结为 **session-frame derivation evidence**。本流程不直接生成 READY map，也不绕过 alignment。

## 1. 边界

```text
Frozen source bag
  -> preflight hash / platform / workspace audit
  -> ManageMappingSession OP_START
       start_sensor=false
       start_chassis=false
       use_sim_time=true
       derived mapping bag recording remains enabled
  -> source-bag replay
  -> FAST-LIVO2 + existing mapping pipeline
  -> ManageMappingSession OP_FINALIZE_CAPTURE
  -> CANDIDATE_READY
  -> agt_offline_assets ingest_mapping_session
  -> PROCESSING workspace evidence
```

MappingSession 仍拥有 mapping stack、derived mapping bag、grid save、normal stop、PCD readiness 和 static-candidate quality gate。`agt_offline_assets` 不启动 FAST-LIVO2，也不发布 ROS topic/TF/velocity。

当前 V25-09A 的 source replay 由一次性 `agt_system_manager/replay_mapping_to_workspace.py` 子进程执行，因为现有 Experiment Manager 仍把 playback 与 active recording 视为互斥操作，而 MappingSession 的现有 static-candidate builder 又需要同步记录 `/agt/mapping/odometry` 和 `/agt/mapping/registered_points` 的 derived bag。这个 bridge 不增加公共 ROS interface；后续可在 Experiment Manager 支持受控 `1 playback + 1 recording` 并发后切回 `/agt/data/bags/manage`，不改变 workspace/ingestion contract。

## 2. 前置条件

先完成 `init-map`，得到 PROCESSING workspace：

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py init-map \
  --maps-root runtime/maps \
  --map-id greenhouse_a \
  --dataset dataset_binding.yaml \
  --recipe recipe.yaml \
  --site-frame site_frame.yaml \
  --alignment alignment.yaml \
  --platform-profile profiles/platforms/bunker.yaml \
  --calibration calibration.yaml
```

预检会再次要求：

- workspace 当前为 `DRAFT` 或 `PROCESSING`
- workspace `validate-map` 通过
- source bag 是完整 rosbag2 directory
- source bag bundle SHA-256 与 Dataset binding 完全一致
- platform profile SHA-256 与 workspace lineage 完全一致
- workspace 尚未 ingest 过 MappingSession evidence

任何一项不满足都必须在 ROS mapping/playback 启动前 fail closed。

## 3. 自动 replay

先启动正常的系统管理节点，使下列接口可用：

```text
/agt/system/change_mode
/agt/mapping/manage_session
/agt/data/bags/manage
/agt/maps/manage
```

然后执行：

```bash
ros2 run agt_system_manager replay_mapping_to_workspace.py \
  --workspace-manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --source-bag /absolute/path/to/source_rosbag2 \
  --platform-profile profiles/platforms/bunker.yaml \
  --rate 1.0 \
  --settle-seconds 2.0
```

如果当前 mapping profile 需要显式 user config：

```bash
... --user-config-path /absolute/path/to/user_config.yaml
```

执行过程会依次打印：

```text
PREFLIGHT_PASS
MAPPING_READY
INGESTED_PROCESSING_EVIDENCE
```

失败发生在 MappingSession finalize 之前时，CLI 会请求 `OP_DISCARD` 回收受管 mapping session；若 finalize 已成功但 ingestion 失败，则保留 `CANDIDATE_READY` session，便于修复 lineage 后重新 ingest，不主动删除证据。

## 4. 输出

成功后 workspace 新增：

```text
processing/mapping_session/
  handoff.yaml
  evidence/
    session.yaml
  candidate/
    ground_temporal.yaml
    ground_temporal.pgm
    comparison_report.json        # 若 MappingSession 提供
  localization/
    localization_map.pcd
    localization_map.processing.yaml

processing/source_replay.log
```

`handoff.yaml` 至少冻结：

- source Dataset identity / source bag hash
- MappingSession identity/state
- derived mapping bag bundle hash
- session file hash
- candidate map YAML/PGM hash
- localization PCD / processing-record hash
- source frame=`mapping_session`
- canonical frame=`map`
- `materialized=false`

这些证据也进入 map manifest `assets` 与 `map_content_sha256`。

## 5. 为什么此时不能 READY

MappingSession 的 PCD 与 candidate map 仍处于 session frame。即使 `alignment/alignment.yaml` 已经定义 `mapping_session -> map`，ingestion 也不会偷偷把原始文件复制成：

```text
navigation/map.yaml
navigation/map.pgm
pointcloud/localization_map.pcd
```

尤其在同一设施场所不同生长时期，alignment 往往不是 identity。若跳过刚体变换和重投影，就会产生“manifest 声称同一 map frame，但 PCD/PGM 实际仍在不同 session frame”的伪合规地图。

因此下一阶段固定为：

```text
PROCESSING mapping-session evidence
  -> apply/verify alignment
  -> canonical PCD materialization
  -> canonical OccupancyGrid reprojection/build
  -> cleaning / ground / stable-structure products
  -> alignment_report + map_quality_report
  -> semantic editing
  -> refresh-map --state READY
```

READY 之后内容不可原地修改；新的对齐、清洗或季节数据必须创建新的 map version。
