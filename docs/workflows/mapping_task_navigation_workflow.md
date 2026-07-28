# 建图、任务编排与导航统一流程

## 状态顺序

```text
START
  -> MAPPING
  -> Ctrl+C / FINALIZE_CAPTURE
  -> WAITING_ASSETS
  -> BUILDING_STATIC_MAP
  -> CANDIDATE_READY
  -> Qt candidate 原位编辑
  -> COMMIT
  -> READY map version + tasks/
  -> Qt offline 任务编排和 Nav2 规划预览
  -> 激活地图版本
  -> NAVIGATION
  -> ExecuteWaypointTask
```

`READY` 底图不可修改。即使分辨率、边界、尺寸和 origin 不变，像素修改也只能发生在
`runtime/mapping_sessions/` 的候选文件中；`COMMIT` 会重新校验候选并生成新版本。

## 一次命令完成采集和 Qt 编排

先启动常驻系统管理器：

```bash
source install/setup.bash
ros2 launch agt_system_manager system_manager.launch.py runtime_dir:=runtime
```

另一个终端启动受管建图。实时采集完成时只按一次 `Ctrl+C`：

```bash
source install/setup.bash
ros2 run agt_system_manager mapping_session_workflow.py run \
  --map-id greenhouse_test_01 \
  --qt-authoring \
  --activate-after-edit
```

该命令执行以下操作：

1. 启动 mapping profile，并强制录制 `mapping` bag。
2. 第一次 `Ctrl+C` 调用 FINALIZE_CAPTURE，不直接终止 launch。
3. 在地图仍在线时保存 trinary PGM/YAML；确认图中同时存在 free/occupied 栅格后才正常停止
   建图，让 PCD 和 bag 收口。全 unknown 在线图会报错并保持建图运行。
4. 把在线图固定到 `online_preview/`，从本次 bag 的注册点云、里程计和静态 TF 离线重建
   射线 free/unknown 基线，再叠加 `ground_temporal` 重复障碍证据和完整多边形车体扫掠。
   新画布外扩区域保持 unknown。位姿/地面拟合失败、证据裁剪、报告不一致或有效区域触碰
   保护边界时进入 `CANDIDATE_BUILD_FAILED`，不能打开 Qt；再次执行 FINALIZE 只重试离线阶段。
5. 离线报告通过后才进入 `CANDIDATE_READY` 并打开 `candidate` Qt profile。它只能编辑并
   原位保存本次候选，不能打开其他地图或另存为。
6. 关闭候选编辑器后执行 COMMIT，复检生产画布与边界并生成新地图版本；
   `--activate-after-edit` 会同时激活该版本。
   维护版 Qt 保存 YAML 时会省略可选 `mode` 键；COMMIT 只对键缺失的情况原子补回
   `mode: trinary` 并记录修复，显式 `scale`、`raw` 或空值仍拒绝提交。
7. 启动 planner-only `waypoint_preview.launch.py` 和 Qt `offline` Task Library。任务保存到新版本的
   `tasks/`，预览可绕开占据格，但不启动车辆。

结束规划预览时按 `Ctrl+C`。该阶段没有 controller、BT navigator、定位、安全使能或底盘节点。

## 分步操作

不使用自动 Qt 编排时：

```bash
ros2 run agt_system_manager mapping_session_workflow.py run --map-id greenhouse_test_01
ros2 run agt_system_manager mapping_session_workflow.py status
ros2 run agt_system_manager mapping_session_workflow.py finalize \
  --session-id <session_id> --timeout 300
ros2 run agt_system_manager mapping_session_workflow.py commit \
  --session-id <session_id> --activate
```

`run` 只在离线静态图门禁通过后返回 `candidate_map_yaml`、`localization_pcd` 和
`bag_directory`；`commit` 返回
`map_version_id`、`registered_map_yaml` 和 `tasks_directory`。放弃测试会话使用可恢复回收：

```bash
ros2 run agt_system_manager mapping_session_workflow.py discard \
  --session-id <session_id>
```

## 任务编排规则

1. 在 Task Library 新建任务。
2. 每个点先点位置、再点方向。相邻点的显示连线不是必须直线行驶的轨迹。
3. 保存只要求任务点端点位于可接受栅格；用“预览路径”调用 Nav2 planner 检查实际逐段绕行。
4. 规划失败时调整任务点或地图候选，不通过删除显示连线来规避。
5. 保存任务后关闭离线预览，再启动完整 navigation 模式。执行必须继续通过地图身份、定位、
   TaskReadiness、Nav2 lifecycle 和 `agt_safety` 门禁。

## 测试 bag 证据

建图会话自动录制 `mapping` profile：

- `/clock`、`/tf`、`/tf_static`
- `/agt/sensors/lidar/custom`、`/agt/sensors/imu/data`
- `/agt/mapping/odometry`
- `/agt/mapping/registered_points`、`/agt/mapping/registered_points_lidar`
- `/agt/mapping/octomap_points`（实际送入全图 OctoMap 的限频点云）
- `/agt/map/mapping_occupancy`
- `/agt/chassis/odometry`、`/agt/chassis/status`、`/agt/chassis/connected`

`/agt/map/mapping_occupancy_raw` 是 OctoMap 到项目中继器的内部 volatile topic，不作为
Qt、SaveMap 或测试 bag 接口。

任务重复测试使用 `navigation` bag profile，并至少保留地图/代价地图、定位状态、项目导航状态、
任务 Action feedback/status、安全状态/急停、速度链、底盘里程计/状态、系统健康、TaskReadiness 和
`/diagnostics`。每次测试还应记录：Git commit、地图 ID/version/hash、任务 JSON/hash、平台 profile、
循环次数、是否实车、开始/结束时间、Action 最终结果、missed waypoint、人工干预和急停事件。

详细任务文件与执行约束见
[qt5_offline_task_group_editor.md](qt5_offline_task_group_editor.md)。
