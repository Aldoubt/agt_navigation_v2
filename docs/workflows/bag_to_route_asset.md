# Bag -> Map Version -> Semantic Map -> Route Asset Workflow

本工作流是 Offline Asset & Evaluation Plane 的推荐执行顺序。目标不是规定唯一算法，而是保证任何场景都按同一 lineage 生成可审计资产。

## 0. 输入冻结

准备：

```text
managed bag / dataset binding
calibration set
platform profile
repository commit
derivation recipe
```

检查：

```text
bag hash
required topics
message timestamps
static TF / calibration identity
platform profile hash
```

输出：

```text
source/dataset_binding.yaml
derivation/recipe.yaml
```

在输入 identity 改变后不得继续沿用旧派生 hash。

## 1. Replay mapping

统一用 bag clock 重放 raw input，不重放已经录入 bag 的旧 `/tf`、FAST-LIVO2 output、registered cloud 或旧 occupancy 来制造双 publisher。

典型输入：

```text
/agt/sensors/lidar/custom
/agt/sensors/imu/data
/tf_static       # 仅当来源受控且与 Calibration Set 一致
```

运行当前 mapping pipeline，得到：

```text
raw trajectory
registered cloud
raw_map.pcd
```

如使用 offline trajectory optimization，必须把 backend/config hash 和优化前后轨迹都保留。

## 2. Canonical site alignment

选择模式：

```text
ENU_GEOREFERENCE       # evaluation/open-field
SITE_CONTROL_POINTS    # greenhouse/facility
REFERENCE_MAP          # same site, later epoch
```

生成：

```text
alignment/site_frame.yaml
alignment/alignment.yaml
alignment/alignment_report.json
```

必须先确定 canonical site frame，再派生 PCD/Occupancy/Semantic/Route。禁止后面对每种资产分别手动对齐。

对于同一场景不同生长期：

```text
new bag
 -> new epoch_id
 -> new map_version_id
 -> align stable/control-point evidence to same site frame
 -> compare against previous accepted version
```

不要让叶片、软枝和临时障碍成为跨时期 alignment 的主要约束。

## 3. Map cleaning

建议顺序作为默认 recipe 起点：

```text
raw_map.pcd
 -> workspace crop
 -> voxel downsample
 -> statistical/radius outlier removal
 -> ground/terrain classification
 -> stable-structure selection (when available)
```

所有参数写入 `processing/cleaning.yaml`。人工 crop/delete 必须导出 patch artifact。

输出至少：

```text
pointcloud/cleaned_map.pcd
pointcloud/localization_map.pcd
pointcloud/localization_map.processing.yaml
reports/ground_report.json
```

## 4. Global navigation map

由已对齐 cleaned/ground-obstacle product 生成：

```text
navigation/map.pgm
navigation/map.yaml
```

Global Navigation Map 只表达长期二维几何，不写入 temporary obstacle。其 origin/resolution/frame 必须与 map version 的 canonical `map` identity 一致。

## 5. Map quality gate

运行 map validation：

```text
hash check
PCD processing record
alignment quality
shared map frame
PGM/YAML consistency
asset extents
semantic binding (if present)
```

输出：

```text
reports/map_quality_report.json
```

只有通过 gate 的 version 才能进入 READY。当前 registry 的基础文件/hash 验证仍保留；新增 quality report 是派生流程的更高层 acceptance evidence。

## 6. Semantic editing

以 READY/候选 Global Navigation Map 为只读底图，在 Semantic Editor 中编辑：

```text
field_boundary
exclusion_zone / keepout_zone
row_centerline
access_lane
headland_zone
entry_pose
work_direction
SemanticWaypoint
```

地图 hash 不一致时保持只读。语义编辑不修改 PGM/PCD。

保存：

```text
semantic/semantic_map.geojson
semantic/coverage.yaml
semantic/validation_report.json
```

## 7. Route rule selection

选择一个 versioned `policy.yaml`，例如：

```text
annotated_rows
crop_centerlines
use access lanes
headland connection
allow reverse
clearance-aware cost
curvature / direction-change penalty
```

Vehicle geometry 读取 `profiles/platforms/<platform>.yaml`，不从 policy 手抄 footprint 或 wheelbase。

## 8. Offline route derivation

输入：

```text
Global Occupancy
+ Semantic Map
+ Route Policy
+ Vehicle Profile
```

planner 可以是：

```text
Fields2Cover / Reeds-Shepp
Smac Hybrid-A*
State Lattice
custom semantic route planner
```

统一输出 Route Asset：

```text
route.yaml
route.csv
policy.yaml
preview.geojson
feasibility_report.json
```

Route 允许由语义规则自动派生，而不是要求人工画所有密集 waypoint。

## 9. Feasibility preview

预览必须叠加：

```text
occupancy
semantic layers
route centerline
forward/reverse segments
sampled real navigation footprint
minimum-clearance hot spots
invalid/collision poses
stop/event anchors
```

只有 centerline 可视化不算 passability evidence。

推荐 UI 操作循环：

```text
Generate
  -> Visualize
  -> Inspect footprint/clearance
  -> Apply non-destructive tuning
  -> Re-sample
  -> Re-run feasibility
  -> Save new route revision
```

## 10. Route tuning

微调只生成 `tuning.yaml` 或新的 policy 参数，不修改 READY base route。每次改变路线几何必须重新生成：

```text
route.csv
route hash
feasibility report
preview
```

直到：

```text
status = READY
footprint_collision_count = 0
kinematic violations = 0
semantic ERROR = 0
```

具体 clearance warning/pass threshold 由 site/policy 配置确定，不在通用合同写死。

## 11. 后续 runtime 接入

本阶段生成的 READY Route Asset 后续由 ROUTE backend 读取：

```text
Route Asset (map)
 -> active segment
 -> map-to-odom transform snapshot
 -> Runtime Path (odom)
 -> Vehicle Tracker Adapter
 -> Controller
 -> Safety
```

采摘、喷药、对点拍照等业务只通过 route `event_ref`/semantic anchor 与 Mission/BT 关联；Route Executor 本身不执行这些业务。

## 12. 最小复现实验记录

每一次正式复现至少记录：

```text
dataset_id / bag hash
calibration_id / hash
platform profile hash
repository commit
recipe hash
site_id / epoch_id
alignment method + report
map version + manifest hash
semantic map hash
route policy hash
route revision + hash
feasibility report
```

有这些信息时，另一台工作站应能从原 bag 重新得到同 lineage 的地图和路线产品，或明确指出算法/依赖版本导致的非确定性差异。
