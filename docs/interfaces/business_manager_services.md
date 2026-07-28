# 地图与 Bag 业务接口

本页定义第一版 map/experiment manager ROS 2 facade。Qt、Web 和其他编排节点是客户端，不能
直接构造 `MapRegistry`、`ExperimentManager`，不能读取 active pointer，也不能持有 rosbag 进程。

## 地图

| 接口 | 类型 | 语义 |
| --- | --- | --- |
| `/agt/maps/list` | `ListMapVersions` service | 按 `map_id`、枚举 state 过滤；默认不返回 deleted |
| `/agt/maps/manage` | `ManageMapVersion` service | get active、validate、activate、pin/unpin、archive、soft delete、purge、import candidate |
| `/agt/maps/active` | `MapVersionSummary` topic | 唯一活动地图上下文；reliable transient-local depth 1 |

`OP_IMPORT_CANDIDATE` 只供项目受管建图后端使用。请求给出已通过建图质量门禁的 YAML、PCD、
processing record 和平台 profile；manager 将其复制进新版本，重新计算/校验 hash，并返回受管的
`navigation_yaml`、`localization_pcd`、`processing_record` 和 `tasks_directory`。客户端不得根据
`runtime_dir` 自行拼接这些路径。

操作成功表示请求完成；版本是否可激活仍由 `MapVersionSummary.valid`、`state` 和 validation 字段
表达。activate 只接受 hash 完整且有效的 READY 版本。soft delete/purge 需要
`confirm_destructive=true`，并拒绝 active、pinned、processing、parent dependency 或实验引用。

稳定错误码为 `ERROR_NONE`、`ERROR_NOT_FOUND`、`ERROR_INVALID_REQUEST`、`ERROR_CONFLICT`、
`ERROR_VALIDATION_FAILED`、`ERROR_CONFIRMATION_REQUIRED` 和 `ERROR_INTERNAL`。

## Bag 与实验

| 接口 | 类型 | 语义 |
| --- | --- | --- |
| `/agt/data/bags/list` | `ListBagSessions` service | 列出 runtime 内完整 Bag，支持 state/experiment 过滤 |
| `/agt/data/bags/manage` | `ManageBagSession` service | status、录制/回放启停、实验创建/完成/中断 |
| `/agt/data/bags/status` | `BagSessionSummary` topic | 当前 recorder/player/experiment 快照；reliable transient-local depth 1 |

录制只接受 `bag_profiles.yaml` 中显式、非空、全为绝对 topic 名的 profile；`-a` 无效。回放只接受
manager 列出的相对 `bag_id`，rate 有界为 `[0.1, 4.0]`，且 `simulation=true`。路径必须保持在
配置的 runtime root 下。

创建实验时可绑定 Mission ID/version/hash、地图 ID/version/hash，并给出 platform、calibration、
Nav2 配置文件作为快照输入。持久化的 `RUNNING` 实验在进程重启后变为 `INTERRUPTED`。recorder 或
player 意外退出发布 `STATE_ERROR`，不会显示为完成或正常 IDLE。

稳定错误码为 `ERROR_NONE`、`ERROR_NOT_FOUND`、`ERROR_INVALID_REQUEST`、`ERROR_CONFLICT`、
`ERROR_PROFILE_INVALID` 和 `ERROR_INTERNAL`。

## 建图组合

`ManageMappingSession` 保留为有限编排 Action：

```text
START
  -> create experiment
  -> start mapping bag profile
  -> start mapping capability with record_bag=false

FINALIZE_CAPTURE
  -> save online preview
  -> stop mapping and flush PCD
  -> stop bag and complete experiment
  -> validate artifacts and build offline candidate

COMMIT
  -> import candidate through map manager
  -> optionally activate through map manager
```

任一步骤失败都保留显式 session 状态和错误；不静默成功，也不由 Qt/Web 接管子进程或 registry。
