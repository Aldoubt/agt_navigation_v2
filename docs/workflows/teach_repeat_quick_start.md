# 示教-复现快速流程

## 1. 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_interfaces agt_experiment_manager agt_teach_repeat --symlink-install
source install/setup.bash
```

## 2. 从已有 bag 提取路径

先确认 bag 中已有 FAST-LIVO2 输出 `/agt/mapping/odometry`。只有 raw MID360/IMU 的 bag 必须先通过
FAST-LIVO2 重放生成 mapping output；提取器不会回退到 cmd_vel 或 chassis odometry。

```bash
ros2 launch agt_teach_repeat teach_extract.launch.py \
  bag:=/absolute/path/to/bag \
  demo_id:=greenhouse_route_001 \
  output_root:=/absolute/path/to/runtime/teach_repeat \
  map_id:=teach_base_v1 \
  map_yaml:=/absolute/path/to/map.yaml \
  localization_pcd:=/absolute/path/to/localization_map.pcd \
  processing_record:=/absolute/path/to/localization_map.processing.yaml \
  platform_profile:=/absolute/path/to/profiles/platforms/bunker.yaml \
  map_from_teach_odom_x:=0.0 \
  map_from_teach_odom_y:=0.0 \
  map_from_teach_odom_yaw:=0.0
```

即使当前测得是单位变换，也应把三个值显式记录。重复 demo 默认拒绝；只有确认替换时传
`overwrite:=true`。

## 3. 离线预览

```bash
ros2 launch agt_teach_repeat teach_preview.launch.py \
  manifest:=/absolute/path/to/runtime/teach_repeat/greenhouse_route_001/manifest.yaml
```

该 launch 只启动 map server、publisher、validator、corridor auditor 和可选 RViz。它不启动
controller、定位、safety enable、底盘或任何速度发布者。检查：

```bash
ros2 topic echo --once /agt/teach/status
ros2 topic echo --once /agt/teach/validation_report
ros2 topic echo --once /agt/teach/corridor_report
```

hash 不匹配时仍可看预览，但 `eligible_for_execution=false`。必须解决 mismatch 并重新验证，不能用
预览成功代替资产绑定。

## 4. 实车低速复现

先单独启动完整 navigation，确认 map/localization/局部障碍/Collision Monitor/安全链均正常。再启动：

```bash
ros2 launch agt_teach_repeat repeat_test.launch.py \
  manifest:=/absolute/path/to/runtime/teach_repeat/greenhouse_route_001/manifest.yaml \
  execution_enabled:=true
```

默认 `auto_start:=true`，但只有全部门禁就绪才发送 child goal。要保留人工二次确认，使用
`auto_start:=false`，然后调用：

```bash
ros2 service call /agt/teach/start std_srvs/srv/Trigger '{}'
ros2 service call /agt/teach/cancel std_srvs/srv/Trigger '{}'
```

必须按顺序确认：

1. navigation 已启动且 Nav2 lifecycle active。
2. `/agt/localization/status` 为 TRACKING、accepted、pose_valid、ERROR_NONE 且新鲜。
3. `/agt/perception/obstacle_cloud` 正常，local costmap 和 Collision Monitor 正常。
4. 硬件/软件急停可用且测试通过。
5. `/agt/system/task_readiness.ready=true` 且 map identity 匹配。
6. 操作者显式使能 motion；示教执行器不会代为使能。
7. 首次把有效速度限制在 0.15-0.20 m/s。
8. 人员不得站在预计运动区域，现场必须有人可立即急停。
9. 历史上曾通过不代表当前无障碍，实时障碍链不得关闭。

## 5. 录包和结果

选择显式 profile：

```bash
ros2 launch agt_bringup bag_record.launch.py \
  bag_profile:=teach_repeat \
  runtime_dir:=/absolute/path/to/runtime
```

每次运行结果写入 manifest 同目录的 `runs/<run_id>/`。若已有 RUNNING Experiment Manager session，
把 `experiment_root` 和 `experiment_id` 传给 `repeat_test.launch.py`，评测器会调用现有 manager 的
teach-repeat 记录接口。

## 6. 地图定位离线等级

```bash
ros2 run agt_teach_repeat localization_map_evaluator \
  --manifest /absolute/path/to/manifest.yaml \
  --samples-json /absolute/path/to/relocalization_attempts.json \
  --config /absolute/path/to/teach_repeat.yaml \
  --output /absolute/path/to/localization_evaluation.json
```

等级为 `INVALID/OFFLINE_ONLY/FIELD_CANDIDATE/FIELD_VALIDATED`。没有足够次数时不会因单次低 fitness
升级；未填充的后端指标保持 null。
