# BUNKER RTAB-Map 坡地导航设计迁移说明

本分支 `feat/bunker-rtabmap-slope-nav` 不再继续实现 BUNKER 真机 Runtime 功能。

2026-08-27 起，BUNKER + MID360 + FAST-LIVO2 + RTAB-Map + GNSS + Nav2 + 坡地局部感知的正式实现迁移到：

```text
Aldoubt/agt_navigation_runtime
branch: feat/bunker-rtabmap-slope-nav
```

仓库职责冻结为：

- `agt_navigation_v2`：离线 Site Asset 生产，包括地图处理、语义地图、Keepout、路线/任务资产、版本与 READY 部署包。
- `agt_navigation_runtime`：机器人实际运行，包括传感器、连续里程计、定位/融合、局部感知、Nav2、Mission/BT、Safety、Chassis。

原设计文档保留为历史记录，不应作为 Runtime 实现真源。Runtime 侧设计文档是：

```text
docs/superpowers/specs/2026-08-27-bunker-rtabmap-slope-nav-design.md
```
