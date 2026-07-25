# Runtime 数据迁移与优化对比记录

更新时间：2026-07-25。

本文记录 BUNKER MID360/FAST-LIVO2 建图链本次代码修改、离线验证结果以及
`runtime/` 数据的迁移边界。代码、配置和测试进入 Git；rosbag、PCD、PGM、日志和
构建产物留在外部数据归档中，不直接提交到 GitHub。

## 本次工作结论

当前完成的是可回放、可对比的 BUNKER 建图 baseline，尚未完成实车标定和导航执行验收：

- FAST-LIVO2 前增加了 profile 驱动的 Livox CustomMsg 车体/高台自滤除，输出字段和
  点顺序保持，TF 不可用时 fail-closed；同时单独拒绝 `(0, 0, 0)` 等无效点，避免把
  无效占位点误计为车体自滤除。
- 从 `mapping_20260719_172810` 估计出当前外参候选：雷达中心相对
  `base_footprint` 地面约 `0.607 m`，roll 约 `+0.37 deg`，ROS pitch 约 `+23.2 deg`。
  其中 `base_link` 高度和机械实测仍未确认，配置继续保留 `calibration_verified: false`。
- PCD 持久化使用稀疏有符号 64 位体素键，过滤非有限点和绝对坐标越界点，并写入
  `state: ready` 和 PCD SHA-256。当前优化 PCD 可用于后续内容校验，但不代表已通过实车
  定位验收。
- 全图 OctoMap 改为有界副本：默认 `0.2 Hz`、`0.10 m` 输入体素压缩、每次最多 `8,000`
  点，最大射线范围从 `40 m` 收紧到 `15 m`，并开启 2D 增量投影和压缩。该节流只保护
  全图 OctoMap，不改变 FAST-LIVO2 和局部障碍链的注册点云输入；PCD 回放可显式关闭
  OctoMap。

验证结果：本次代码构建通过，相关 Python 回归测试 `24 passed`，完整已有测试汇总为
`438` 个测试、`0` 个失败、`14` 个跳过。优化后的 2 分钟有效 TF 回放中，OctoMap
进程约 `104 MiB` RSS、节流节点约 `63 MiB`、FAST-LIVO2 约 `149 MiB`，未出现新的
`Message Filter queue is full`。这些是资源回放证据，不是整段 bag 的峰值保证。

## 可复现的旧/新对比

对比输入必须固定为同一原始 bag：
`runtime/rosbag/mapping_20260719_172810`。

| 项目 | 旧 baseline | 本次优化 | 用途 |
| --- | --- | --- | --- |
| 注册点云 bag | 旧 FAST-LIVO2 输出，若仍保留则作为 baseline | `runtime/rosbag/mapping_20260719_172810_optimized_registered`，约 2.72 GiB | 验证外参、自滤除和后处理影响 |
| PCD 处理输入点数 | 56,280,780 | 56,300,535 | 记录处理规模变化 |
| PCD 输出点数 | 392,748 | 390,464 | 观察车辆点和异常点清理后的地图规模 |
| 推荐 PGM | `runtime/maps/mapping_20260719_172810_fast_variants/ground_temporal.pgm` | `runtime/maps/mapping_20260719_172810_optimized_variants/ground_temporal.pgm` | 同一栅格元数据下进行像素差异对比 |
| ground_temporal 占用像素 | 180,465 | 180,558 | 不能单独作为质量结论，只能作为回归指标 |
| 扫掠清理栅格数 | 9,059 | 8,556 | 检查外参和车体清理对自由空间的影响 |
| PCD SHA-256 | 旧记录按旧 processing record | `sha256:a621fa358f3901c32291a52440cecd46e68e10ecc1a2ee8ed9fd88149967cb64` | 版本登记和定位加载校验 |

旧、新 PGM 的 YAML 元数据相同，但 PGM 内容不同：旧文件 SHA-256 为
`5c0605dd9fe6281081f2fe65c3deaf90df56175f255556f630b45ea5d0b7c7c9`，优化文件
SHA-256 为 `453599d96dc183f94f0613789f72b2f7a969f060f6d91da341370dbf48d09b42`。
这说明对比是有效的；要证明优化有效，还应同时查看 RViz 叠加图、车体附近误障碍数量、
远处障碍保留率、规划可行性和定位 fitness，不能只比较文件大小或占用像素数量。

## 建议迁移布局

建议将 runtime 迁移到 Git 工作区之外，例如 `$ARCHIVE_ROOT`，保持原始输入和派生输出
分层保存：

```text
$ARCHIVE_ROOT/
  mapping_20260719_172810_extrinsic_optimization/
    source_bag/                 # 原始 CustomMsg + IMU bag
    baseline/                   # 旧 PCD、旧 PGM 及 processing/report
    optimized/                  # 新 PCD、calibration、processing、result index
    optimized_registered_bag/   # 可重跑 PGM 的注册点云 bag
    variants/                   # ground_only、ground_temporal、layered
    manifest.sha256             # 迁移后重新计算的清单
```

迁移前先设置实际归档位置并复制，不要把归档目录放进 Git 工作区：

```bash
export AGT_WS="${AGT_WS:-$PWD}"
export ARCHIVE_ROOT="/path/to/agt_navigation_v2_runtime_archive"
export CASE_DIR="$ARCHIVE_ROOT/mapping_20260719_172810_extrinsic_optimization"
mkdir -p "$CASE_DIR"
rsync -a "$AGT_WS/runtime/rosbag/mapping_20260719_172810/" "$CASE_DIR/source_bag/"
rsync -a "$AGT_WS/runtime/maps/mapping_20260719_172810_fast/" "$CASE_DIR/baseline/"
rsync -a "$AGT_WS/runtime/maps/mapping_20260719_172810_fast_variants/" "$CASE_DIR/baseline_variants/"
rsync -a "$AGT_WS/runtime/maps/mapping_20260719_172810_optimized/" "$CASE_DIR/optimized/"
rsync -a "$AGT_WS/runtime/maps/mapping_20260719_172810_optimized_variants/" "$CASE_DIR/optimized_variants/"
rsync -a "$AGT_WS/runtime/rosbag/mapping_20260719_172810_optimized_registered/" "$CASE_DIR/optimized_registered_bag/"
find "$CASE_DIR" -type f -print0 | sort -z | xargs -0 sha256sum > "$CASE_DIR/manifest.sha256"
```

原始 bag 是回归基准，建议至少保留一份离线只读归档。若空间不足，优化注册点云 bag
可以在报告确认、PCD 和 PGM hash 均已保存后转入低成本存储，但不要在此之前删除。

## 保留、对比和清理清单

### 必须保留

- `runtime/rosbag/mapping_20260719_172810`：唯一的原始输入基准。
- `runtime/maps/mapping_20260719_172810_fast`：旧结果和旧 processing record。
- `runtime/maps/mapping_20260719_172810_fast_variants`：旧的三种 PGM 对照。
- `runtime/maps/mapping_20260719_172810_optimized`：新 PCD、校准估计、ready 记录和 result index。
- `runtime/maps/mapping_20260719_172810_optimized_variants`：新旧地图可视化和像素对比的主要证据。
- 本文以及 Git 中的配置、launch、源码和测试。runtime 文件本身被 `.gitignore` 排除，
  不会随代码 push。

### 值得保留做历史对比

- `runtime/maps/bunker_large_obstacles_v2` 到 `v5`：如果需要展示车体清理参数逐版变化，
  保留 PGM 和对应报告；它们不是当前同一 bag 的主证据。
- `runtime/maps/bunker_large_traversability_comparison_20260720`：包含地面、时序和高度层
  对照，适合展示“保守可执行性”边界。
- `runtime/rosbag/bunker_mapping_20260719_163246` 和其他独立测试 bag：仅在仍需复现实机或
  旧参数实验时保留；它们不能替代当前源 bag。
- `lidar_static_map_benchmark/`：这是独立 Git 仓库，当前干净且已关联
  `Aldoubt/lidar-static-map-benchmark`，其大体积 `data/` 和 `results/` 已由自己的忽略规则
  排除，不应嵌入主仓库。

### 已确认的重复，可在外部归档校验后清理一个副本

- `runtime/maps/bunker_large_20260719/pcd/all_raw_points.pcd`
  和 `all_downsampled_points.pcd` 完全相同，SHA-256 均为
  `f98d79e56b82ce14a2039488f6a8428ee2159ff0764692204063f9b45f4b75ed`。
  建议保留语义更明确的 `all_downsampled_points.pcd`，删除或移出名为 `all_raw_points.pcd`
  的重复副本；先完成归档 hash 校验。
- `runtime/maps/replay_asan_abi_fixed_20260720/pcd/localization_map.pcd` 和
  `runtime/maps/replay_asan_fixed_20260720/pcd/localization_map.pcd` 完全相同，适合只保留
  一个 ASAN 回放代表；另一个 `replay_asan_identify2_20260720` hash 不同，不能按重复删除。
- 多份零字节 `myeasylog.log` 是 GUI 临时日志，可清理。

### 临时产物，确认不再调试后可清理

- `runtime/maps/octomap_optimized_smoke`：只用于短时资源 smoke，不是最终地图。
- `runtime/maps/mid360_map`、`web_qos_replay`、`web_mapping_test` 以及 replay/ASAN 小目录：
  仅保留仍能复现问题的一份，其余在报告记录后清理。
- `runtime/logs/system_manager/mapping.log` 约 347 MiB：先提取关键时间段、RSS 和错误证据，
  再压缩或移到归档；不要把整份运行日志提交 Git。
- `build/`、`install/`、`log/`：均可由代码重新构建，不属于迁移数据；确认没有现场诊断需要后
  可清理，但它们不是本次 Git 备份的一部分。

### 尚未确认，不要直接删除

- `runtime/maps/first_vehicle_test.zip` 与 `runtime/maps/first_vehicle_test/` 可能是目录归档
  和展开目录，但当前尚未完成内容级 hash 比对；先解压到临时目录并逐文件比较。
- `runtime/rosbag/navigation_20260719_164431` 约 30.5 GiB：虽然不是当前建图证明的必要输入，
  但可能用于导航链验收，不能仅因体积大而删除。
- 任何旧 PGM 版本：即使 YAML 相同，PGM 像素已确认不同，不能按“文件名相近”删除。

本次只完成了清单和代码备份，不自动删除上述数据。删除前应先完成外部归档、清单 hash
校验和一次恢复抽查。

## GitHub 备份边界

本次主仓库提交应包含当前已修改的代码、配置、接口文档、测试和本迁移记录；不包含
`runtime/`、`build/`、`install/`、`log/` 及嵌套 benchmark 的数据。提交后应确认：

```bash
git status --short --branch
git log -1 --oneline
git ls-files runtime/rosbag runtime/maps
```

最后一条不应输出当前被忽略的大型回放和地图文件。主仓库和独立 benchmark 仓库应分别
通过各自的 remote 备份，避免把两个项目的历史混在一个提交中。
