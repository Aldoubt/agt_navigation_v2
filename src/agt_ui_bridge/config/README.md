# config

UI bridge 的版本化默认配置：

- `map_io.yaml`：基础 OccupancyGrid 加载与保存接口；
- `ros_qt5_gui_app.json`：维护版 Qt5 主界面的兼容 V2 topic 映射；实际启动优先使用
  `ros_qt5_gui_mapping.json`、`ros_qt5_gui_candidate.json` 或
  `ros_qt5_gui_navigation.json`。profile 默认中文、系统窗口边框，
  语言选择持久化到运行时配置并在重启后生效；
- `ros_qt5_gui_offline.json`：只允许多点路径预览、禁止真实任务 Action 的离线 profile；
- `semantic_editor.yaml`：独立语义编辑器的显示、作业参数和显式边界净距，不保存车辆几何；
- `semantic_map_server.yaml`：语义服务器输入、基础地图 topic、field 外部策略和 mask 数值；
- `semantic_schema.yaml`：农业语义地图与覆盖任务文件的 1.0 机器可读合同。

运行中产生的 GUI 配置和语义任务文件写入 `runtime/`，不得覆盖这里的默认合同。

`EnableBaseMapEditing` 控制底图编辑；`EnableBaseMapSaveAs` 和 `EnableMapOpen` 分别控制另存为与
打开其他地图。mapping 只监看，candidate 只允许原位保存启动时传入的候选；navigation、offline
和 teach 保持 READY 底图只读。只有 navigation/offline 打开版本化 Task Library。
`EnableOfflinePlanningPreview` 控制 Task Library 的“预览路径”入口。保存校验只检查任务点端点，
不再配置或执行相邻点直线采样。
