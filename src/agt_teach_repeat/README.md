# agt_teach_repeat

该 package 提供独立的示教路径提取、处理、只读预览、完整 footprint 验证、Nav2
`FollowPath` 复现和系统内部重复性评测。它不修改 FAST-LIVO2，不发布 TF/odometry/速度，
不使能 motion，也不启动底盘。

## 接口

输入包括 rosbag2 中的 `/agt/mapping/odometry`、canonical platform profile、同源 map YAML/PCD/
processing record，以及运行期的 global costmap、结构化定位、TaskReadiness、safety 和急停。
输出位于 `/agt/teach/*`，运行资产位于调用者配置的
`runtime/teach_repeat/<demo_id>/`。

提取器同时写入与 reference path 哈希绑定的 `processed/route_annotations.json`。其中显式记录
方向采样间距、普通转弯/宽掉头的独立累计距离窗口及对应角阈值；publisher 将结果作为 transient-local
`/agt/teach/route_annotations` MarkerArray 发布。Qt `teach` profile 只显示这些标注，不自行分类，
也不因此获得执行权限。

执行器是 Nav2 `/follow_path` 的 Action client；`controller_id` 为 `FollowPath`。显式启动/取消
服务为 `/agt/teach/start` 和 `/agt/teach/cancel`（`std_srvs/Trigger`）。两者都不改变 safety
motion enable 状态。

## TF 责任与非目标

本 package 不发布任何 TF。原始示教 odometry 的实际接口为 `odom -> base_footprint`，资产通过
显式 `map_from_teach_odom` 变换到 `map`。`agt_localization` 仍唯一发布 `map -> odom`，mapping
adapter 仍发布 `odom -> base_footprint`。

本阶段不实现多会话/跨生长周期地图融合、自动 PGM 清理、地图/PCD 合并、语义分割、Qt 执行页面、
Web 一键实车执行或独立绝对位置真值。

完整流程见 `docs/workflows/teach_repeat_quick_start.md`，实车门禁见
`docs/testing/teach_repeat_field_test.md`。

Teach Mapping MVP 通过 `agt_system_manager teach_mapping_workflow.py` 复用本 package 的
`extract_demo`、路径处理、manifest 绑定和 full-footprint 审计。组合复扫入口把
`auto_start` 固定为 false，并将外部 `0.02..0.20 m/s` 上限传给执行器；任务进入任一终态时，
执行器会发布 `SpeedLimit=0.0` 清除本次临时限速影响。
