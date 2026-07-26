# Teach Mapping MVP 架构说明

## 职责归属

`agt_system_manager` 负责版本 1 的示教建图 Session 文档和有界离线工作流。它将路径提取与处理委托给 `agt_teach_repeat`，将导航编排委托给 `agt_bringup`，将建图委托给现有 FAST-LIVO2 链路，并通过规范平台 Profile 和共享路径验证器执行完整足迹检查。

该工作流不发布 TF 或速度指令，不启用运动，不替换当前活动地图，不将 Candidate Map 注册为生产地图，也不写入 Bootstrap 资产。

离线预览可选择 Qt `teach` profile。项目端从绑定 reference path 确定性生成方向和转弯/掉头标注，
Qt 只读显示 `/agt/teach/reference_path` 与 `/agt/teach/route_annotations`；显示成功不改变 Session 阶段，
也不能替代地图哈希、完整足迹、未知空间和碰撞验证。

## Session 合同

每个 Session 存储在 `runtime/teach_mapping/<session_id>/` 下：

```text
session.yaml
bootstrap/
teach_route/
rescan/
candidate_map/
reports/
```

大型源资产仅被引用，不会被复制。`session.yaml` 使用 SHA-256 绑定平台 Profile、Bootstrap Map YAML 及图像、定位 PCD、状态为 Ready 的处理记录、原始示教 Bag、提取后的 Manifest、已注册复扫 Bag 和 Candidate 资产。每次文档更新都先写入并刷新临时文件，再以原子方式替换目标文件。

支持的阶段为 `CREATED`、`BOOTSTRAP_READY`、`PATH_EXTRACTED`、`PATH_VALIDATED`、`RESCAN_READY`、`RESCAN_RECORDED`、`CANDIDATE_MAP_BUILDING`、`CANDIDATE_MAP_READY` 和 `FAILED`。更新失败时保留 `last_successful_stage`、已生成数据和结构化错误。Candidate 构建失败后，流程回到 `RESCAN_RECORDED` 恢复边界；再次构建时必须使用新的 Candidate Map 名称，或由操作员明确清理失败目录。

## 复扫边界

`teach_mapping_rescan.launch.py` 会验证 Session，并针对不可变的 Bootstrap Map 启动且仅启动一条导航链路和一条示教复现前端链路。默认配置为：底盘关闭、执行关闭、Bag 录制开启、自动重定位关闭，并且示教自动启动固定为关闭。传给执行器的速度限制在 `0.02..0.20 m/s`，默认值为 `0.10 m/s`；执行器还会采用 Manifest 中更低的限制，并在任何终止结果发生时清除临时 Nav2 速度限制。

进程启动不等于系统就绪。操作员仍须检查定位、Nav2 生命周期、路径验证、障碍物处理、底盘和急停状态，然后才能单独启用运动并调用 `/agt/teach/start`。

## Candidate Map 边界

Candidate Map 生成由一棵短生命周期的离线进程树完成，且明确不属于 System Manager 模式。它以仿真时间启动建图，不启动传感器、底盘、GUI、RViz、健康检查或录制进程。Bag 回放仅包含原始 MID360 CustomMsg 和 IMU，`/clock` 由 rosbag 的 `--clock` 提供。系统先保存 2D 地图，再向建图进程发送 `SIGINT`；关闭过程最多只能升级为 `SIGTERM`，不得使用 `SIGKILL`，以便 FAST-LIVO2 完成增量 PCD 处理记录。

只有 PGM/YAML、PCD 和状态为 Ready 的处理记录全部通过验证后，Candidate 才视为就绪。报告会比较确定性的栅格、PCD、路径中心以及完整足迹扫掠指标，但绝不选择或发布最终地图。
