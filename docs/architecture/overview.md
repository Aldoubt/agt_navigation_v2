# 架构总览

## 分层
- 适配与描述层：`agt_description`、`agt_sensor_adapters`
- 建图定位感知层：`agt_mapping`、`agt_localization`、`agt_localization_fusion`、`agt_perception`
- 地图与规划层：`agt_map_processing`、`agt_navigation`、`agt_coverage_planning`
- 执行与系统边界层：`agt_safety`、`agt_chassis`、`agt_ui_bridge`、`agt_experiment_manager`
- 评测层：`agt_evaluation`

## 当前目标
- 建立统一 TF、topic、状态接口和配置组织方式
- 为后续模块迁移提供稳定包边界
- 完成一次采集同时产出原始 bag、二维导航图和同源定位 PCD 的可复现链路
- 前端只负责任务编写；可靠执行统一经过项目 Action、Nav2、安全层和底盘适配层

## 可迁移的数据与控制边界

```text
MID360 / camera / future sensors
  -> sensor adapters -> raw bag
  -> mapping/localization/perception
  -> metric map products (PCD + Nav2 raster + semantic sidecars)

managed mapping capture
  -> online OctoMap preview (audit input only)
  -> normal PCD + bag shutdown
  -> offline ray-traced free/unknown baseline
  -> ground_temporal obstacles + canonical footprint sweep
  -> quality-gated editable candidate
  -> immutable READY map version

Qt5 / future Web UI / autostart manager
  -> project task Actions
  -> Nav2 planning and control
  -> agt_safety
  -> platform chassis adapter
```

Qt5 是可替换前端，不拥有 Nav2 状态机、TF、安全或底盘命令。当前人工任务 JSON 通过
`ExecuteWaypointTask` 执行；未来自动任务管理器继续调用同一接口。平台差异集中在
`profiles/platforms/`、传感器适配和底盘适配，地图、任务与评测接口保持不变。

## 演进顺序

1. 人工 Demo：建图和原始 bag 同时保存，人工重定位、Qt 选点、项目多点 Action 执行；
2. 稳定运行：启动检查、生命周期管理、任务恢复、日志/数据快照和失败降级；
3. 手工语义地图：基础栅格保持只读来源，语义区域与通道使用独立版本化 sidecar；
4. 感知语义：视觉或点云检测输出动态障碍/语义观测，不直接修改静态底图；
5. 融合规划：经时效、置信度和坐标变换验证后，把人、作物或标志物信息接入局部
   costmap/行为策略，并始终保留几何避障与安全链。

## 本阶段不做
- 第三方算法源码迁移
- 具体导航参数调优
- 无人值守自动任务恢复
- 视觉/点云语义直接控制局部规划
