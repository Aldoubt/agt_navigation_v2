# agt_teach_repeat

该 package 提供独立的示教路径提取、处理、只读预览、完整 footprint 验证、Nav2
`FollowPath` 复现和系统内部重复性评测。它不修改 FAST-LIVO2，不发布 TF/odometry/速度，
不使能 motion，也不启动底盘。

## 接口

输入包括 rosbag2 中的 `/agt/mapping/odometry`、canonical platform profile、同源 map YAML/PCD/
processing record，以及运行期的 global costmap、结构化定位、TaskReadiness、safety 和急停。
输出位于 `/agt/teach/*`，运行资产位于调用者配置的
`runtime/teach_repeat/<demo_id>/`。

执行器是 Nav2 `/follow_path` 的 Action client；`controller_id` 为 `FollowPath`。显式启动/取消
服务为 `/agt/teach/start` 和 `/agt/teach/cancel`（`std_srvs/Trigger`）。两者都不改变 safety
motion enable 状态。

## TF 责任与非目标

本 package 不发布任何 TF。原始示教 odometry 的实际接口为 `odom -> base_footprint`，资产通过
显式 `map_from_teach_odom` 变换到 `map`。`agt_localization` 仍唯一发布 `map -> odom`，mapping
adapter 仍发布 `odom -> base_footprint`。

本阶段不实现多会话/跨生长周期地图融合、自动 PGM 清理、地图/PCD 合并、语义分割、Qt 新页面、
Web 一键实车执行或独立绝对位置真值。

完整流程见 `docs/workflows/teach_repeat_quick_start.md`，实车门禁见
`docs/testing/teach_repeat_field_test.md`。
