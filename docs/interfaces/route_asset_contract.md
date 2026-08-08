# Route Asset, Rule Derivation and Vehicle Feasibility Contract

本文定义从 READY Map Version、Semantic Map 与 Vehicle Profile 派生可审计离线路线资产的正式合同

必须保持

```text
Semantic Map -> Route Policy -> Route Asset -> Runtime Path
SemanticWaypoint != WaypointTask != Route != Runtime Path
```

Route Asset 不是 Mission、SemanticWaypoint Library 或 controller Runtime Path

## 1. 目录

```text
runtime/maps/<map_id>/versions/<map_version_id>/routes/<route_id>/<revision>/
  route.yaml
  route.csv
  policy.yaml
  feasibility_report.json
  preview.geojson
  tuning.yaml              # 可选
```

Route revision 是绑定到 READY Map Version 的独立子资产，新增 revision 不修改已冻结地图内容

## 2. Map compatibility identity

Route 不绑定 `manifest.yaml` 原始文件 SHA，因为 `agt_map_manager` 在 activate、pin、archive 等 Registry 生命周期操作中可能合法修改 `active`、`pinned`、`state` 等管理字段

正式关系是

```text
map_version_id
+
map_content_sha256
```

其中 `map_content_sha256` 只覆盖稳定地图内容与 provenance，包括地图身份、Dataset/Calibration/Recipe/Alignment lineage、Platform binding、navigation metadata 和 frozen asset hashes

以下 Registry 元数据不参与该 identity

```text
state
active
pinned
tags
notes
created_at
```

因此地图激活或 pin 不会让已有 Route 失效，但导航图、Localization Prior、Semantic Map、Coverage、Calibration、Recipe、Alignment 等真实内容变化必须导致 content identity 或独立 asset hash 不匹配

## 3. `route.yaml`

READY revision 最低字段

```yaml
schema_version: 1
route_id: greenhouse_main_route
revision: 1
frame_id: map
map_binding:
  map_id: greenhouse_a
  map_version_id: map_20260810_120000_1234abcd
  map_content_sha256: sha256:<64 lowercase hex>
semantic_binding:
  path: ../../../semantic/semantic_map.geojson
  sha256: sha256:<64 lowercase hex>
  coverage_path: ../../../semantic/coverage.yaml
  coverage_sha256: sha256:<64 lowercase hex>
vehicle_binding:
  platform_id: bunker
  platform_profile_sha256: sha256:<64 lowercase hex>
policy_binding:
  path: policy.yaml
  sha256: sha256:<64 lowercase hex>
route_csv_sha256: sha256:<64 lowercase hex>
feasibility_report_sha256: sha256:<64 lowercase hex>
preview_sha256: sha256:<64 lowercase hex>
status: READY
```

`semantic_map.geojson` 与 `coverage.yaml` 必须都是 READY Map Manifest 已冻结的 canonical assets

派生器不得接受路径相同但 hash 已变化的语义文件

Map content、Semantic、Coverage、Vehicle Profile 或 Route Policy 任一绑定变化都要求新派生或重新验证，不能仅按文件名判断兼容

Route 建立时先写 `DRAFT`，feasibility 与 preview 全部生成后最后一次写 `route.yaml` 才能晋升 `READY`

READY revision 之后禁止原地修改 CSV、policy、preview、feasibility 或 manifest

## 4. Route Policy

`policy.yaml` 描述如何从语义地图派生路线，不复制 Vehicle Profile 的几何真值

```yaml
schema_version: 1
policy_id: greenhouse_inspection_v1
source:
  planning_mode: annotated_rows
  row_interpretation: crop_centerlines
  use_access_lanes: true
  use_headland_zones: true
constraints:
  minimum_clearance_m: 0.15
  allow_reverse: true
  unknown_space_allowed: false
  direction_change_requires_stop: true
costs:
  path_length: 1.0
  clearance: 3.0
  curvature: 2.0
  reverse_distance: 1.5
  direction_change: 4.0
  semantic_corridor_deviation: 5.0
sampling:
  path_resolution_m: 0.05
  footprint_check_resolution_m: 0.05
postprocess:
  smoothing: true
  preserve_stop_anchors: true
```

footprint、vehicle width、wheelbase、track、minimum turning radius 等必须来自 `profiles/platforms/<platform>.yaml`

`footprint_check_resolution_m` 是正式 policy 字段，V25-09A 首版 feasibility backend 仍复用 `agt_coverage_planning.path_validator` 的自适应采样，因此不能把该字段误写成当前 validator 内部每一步采样的直接控制量

## 5. Semantic Map 派生规则

第一版允许使用已有语义

- `row_centerline`：按 `row_interpretation` 解释为道路或作物行中心线
- `access_lane`：显式通行中心线
- `headland_zone`：连接和调头区域约束
- `keepout_zone` / `exclusion_zone`：不可通行
- `entry_pose`：候选入口、起终点
- `waypoint`：命名锚点，可作为任务停顿、定位锚点或路线控制点，但不自动产生执行顺序

派生器只读 Semantic Map，不能反向改写 GeoJSON

### V25-09A `semantic_boustrophedon_mvp`

当前实现支持 `planning_mode=annotated_rows`

- `direct_swaths`：enabled `row_centerline` 直接作为道路，可按 policy 加入 `access_lane`
- `crop_centerlines`：复用现有规则，将相邻作物行中心线确定性派生为行间道路，使用 `preview_aisle_xxx` 作为 derived semantic reference
- 道路按 `work_direction` 法向排序并交替方向形成 boustrophedon 顺序
- 行间 connector 当前仅生成 straight candidate

straight candidate 只是候选，不代表运动学可行

Tracked/skid-steer 车型可能直接通过，Ackermann/nonholonomic 车型若存在原地姿态变化或超过曲率限制，应被 existing minimum-turning-radius gate 拒绝

Hybrid-A*、State Lattice 或 Reeds-Shepp connector backend 后续替换内部生成器，不改变 Route Asset schema

## 6. `route.csv`

CSV 保存高密度几何和运行提示

```csv
seq,segment_id,x,y,yaw,direction,v_ref,curvature,clearance,semantic_ref,event_ref
0,s000,1.000,2.000,0.000,F,0.30,0.000,1.20,row_01,
1,s000,1.050,2.000,0.000,F,0.30,0.000,1.18,row_01,
2,s001,1.100,2.000,3.142,R,0.15,-0.30,0.90,headland_01,tomato_stop_003
```

约束

- Route Asset 当前正式 frame 为 `map`
- `direction` 只允许 `F` / `R`
- 方向切换发生在 segment boundary，policy 可要求先停车
- `v_ref` 是建议值，不绕过 controller/safety 上限
- `semantic_ref` 可引用源 Feature 或由冻结规则确定性产生的 `preview_aisle_xxx`，匿名未知 ID 必须拒绝
- `event_ref` 只引用任务/语义锚点，Route Executor 不自行执行采摘、喷药、拍照等业务

当前 MVP 只自动生成 `F` segment，`R` 已作为合同冻结，reverse-aware connector/planner 留给后续实现

## 7. Vehicle Feasibility

Route 进入 READY 前必须做完整 footprint sweep，不能只检查中心线

输入

```text
Global Navigation Map
+ semantic field / keepout / exclusion
+ Route samples
+ canonical Vehicle Profile navigation_footprint
+ kinematic limits
```

至少检查

- footprint collision
- minimum clearance
- unknown-space intersection
- curvature / minimum-turning-radius violation
- reverse permission
- direction-change feasibility
- full footprint 保持在 enabled `field_boundary`
- full footprint 不进入 `keepout_zone` / `exclusion_zone`
- semantic reference validity
- task stop pose feasibility，在对应 event/stop 存在时

V25-09A 复用 `agt_coverage_planning.path_validator` 的 OccupancyGrid、full-footprint、unknown-space、clearance 和 minimum-turning-radius 核心，再叠加 Semantic Map 的 field/exclusion/keepout full-footprint gate

任何 ERROR 级碰撞、语义可通行域或运动学违规都不能 READY

`feasibility_report.json` 至少包含

```json
{
  "status": "PASS",
  "route_id": "greenhouse_main_route",
  "revision": 1,
  "checks": {
    "full_footprint_sweep": "PASS",
    "semantic_free_space": "PASS",
    "kinematics": "PASS"
  },
  "metrics": {
    "length_m": 42.1,
    "reverse_length_m": 3.2,
    "min_clearance_m": 0.41,
    "direction_changes": 2,
    "footprint_collision_count": 0,
    "semantic_footprint_violation_count": 0,
    "curvature_violation_count": 0,
    "unknown_intersection_count": 0
  },
  "errors": [],
  "warnings": []
}
```

## 8. 可视化

`preview.geojson` 是离线 UI/RViz/Qt/Web 预览产品，也是 READY Route 的冻结验收证据之一

建议图层

```text
base occupancy reference
semantic rows / lanes / headlands / keepout
route centerline
forward/reverse segments
sampled vehicle footprint polygons
clearance hot spots
stop/event anchors
invalid footprint samples
```

预览必须使用与规划相同的 canonical `navigation_footprint`

首版至少输出 route segment、采样 footprint、event anchor，以及 Occupancy/kinematic 或 semantic-free-space invalid footprint

READY revision 的正式 preview 不允许原地重新生成，显示端读取现有 preview；需要改变正式预览证据时创建新 revision 并重新验收

## 9. 微调

允许离线人工/规则微调，但必须 non-destructive

```yaml
schema_version: 1
base_route_sha256: sha256:<64 lowercase hex>
operations:
  - type: lateral_offset
    segment_id: s003
    value_m: 0.10
  - type: speed_scale
    segment_id: s005
    value: 0.7
```

V25-09A 首版支持 `lateral_offset` 和 `speed_scale`

任何几何微调后必须

```text
new revision
 -> 重新读取/采样
 -> footprint sweep
 -> semantic gate
 -> feasibility + preview
 -> new hashes
```

禁止在 GUI 中拖动 READY CSV 点后直接覆盖

## 10. Planner baseline 与创新边界

Route policy 可以调用 Hybrid-A*、State Lattice、Fields2Cover/Reeds-Shepp 或自研 planner

统一 Route Asset/feasibility lineage 与 planner backend 解耦

研究创新可集中在

- semantic corridor / row / headland constraints
- clearance-aware objective
- task-stop-aware routing
- forward/reverse penalty 与 direction change
- 多车型 feasibility comparison
- 多时期地图下 route reuse / revalidation

不把单纯“支持倒车”或“考虑 footprint”作为唯一创新

## 11. Runtime boundary

后续 ROUTE backend 只消费 READY Route Asset，将 active segment 转为 `odom` frame Runtime Path

Route Asset 不发布速度、不拥有 TF、不绕过 project Navigation Capability
