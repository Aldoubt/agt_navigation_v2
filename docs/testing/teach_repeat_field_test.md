# 示教-复现实车测试

## 测试前置

- 使用新 demo ID 和不可变 READY map 版本，不覆盖旧地图、PCD 或 demo。
- 对照 manifest 复核 map YAML、PCD、processing record、reference path SHA-256。
- 确认 `map_from_teach_odom` 来自同源会话测量，不把默认零值当作自动标定。
- 在 RViz 检查路径 frame、起终点、方向、full-footprint 冲突和未知区。
- 架空履带先验证 FollowPath 接收/取消、速度上限、定位失效、TaskReadiness 失效、safety 失效和急停。
- 确认 Nav2、Collision Monitor、`agt_safety` 和 chassis watchdog 都能在独立故障下归零。

## 场地步骤

1. 清场并设置物理隔离，指定一名驾驶/急停人员和一名记录人员。
2. 启动 navigation，不启动 teach executor；完成重定位并观察稳定 TRACKING。
3. 检查 MID360 原始/过滤输入、registered cloud、local obstacle cloud、local costmap 和 Collision Monitor。
4. 以 `execution_enabled=false` 启动 repeat launch，确认路径验证和所有 hash。
5. 设置 `auto_start=false`，操作者显式使能 motion 后调用 `/agt/teach/start`。
6. 第一轮不超过 0.15 m/s；验证直线、缓弯、原地转向和终点停车。
7. 人工触发一次 cancel，再分别模拟定位 stale、TaskReadiness false 和 safety disable，确认 child goal 取消。
8. 在安全条件下验证急停；复位后保持 motion disabled，不能自动恢复任务。
9. 无门禁异常后最多提高到 0.20 m/s，至少完成三次相同方向复现。
10. 正常停止并保存 bag、run metrics、Experiment Manager result 和操作员记录。

## 验收记录

每轮记录 completion、lateral mean/RMSE/P95/max、yaw RMSE/P95/max、duration、tracking lost、degraded、
emergency stop、manual intervention 和 Collision Monitor stop。建议默认候选门槛来自
`config/teach_repeat.yaml`，但它们不是安全认证阈值。

报告必须保留以下声明：轨迹指标使用机载定位估计，只衡量系统内部重复性，不是独立绝对位置真值。
需要绝对精度结论时必须另接 RTK、全站仪、motion capture 或其他独立真值，并定义时间同步/外参。

## 主要风险

- 示教期间可通行不代表复现时无障碍；走廊审计不能清图或关闭实时检测。
- 错误 `map_from_teach_odom` 可生成形状正确但位置整体错误的路径。
- 同源定位用于评测会掩盖共同漂移，不能证明绝对重复性。
- 温室近重复结构可能产生错误接受或歧义，单次低 fitness 不足以批准地图。
- PCD/map/path 在启动后被替换会触发取消，但运行资产仍应由外部流程保持不可变。
- 履带滑移会使 chassis odometry 和 LIO 分离，需同时保留两者诊断。
- speed limit 只是更保守的 Nav2 上限，不替代控制器、Collision Monitor、safety 和 chassis watchdog。
- 进程崩溃、DDS 延迟、Action cancel 延迟和 CAN 断连必须分别做故障注入。

实车结果通过前，模块状态只能是离线完成或 field candidate，不能标记为安全认证或闭环验收完成。
