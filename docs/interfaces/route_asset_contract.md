# Route Asset, Rule Derivation and Vehicle Feasibility Contract

本合同定义如何从已绑定的 Semantic Map、Global Navigation Map 与 Vehicle Profile 派生可审计的离线路线资产。Route Asset 是可版本化导航输入，不是 Mission，不是 SemanticWaypoint Library，也不是 controller Runtime Path。

必须保持：

```text
Semantic Map -> Route Policy -> Route Asset -> Runtime Path
SemanticWaypoint != WaypointTask != Route != Runtime Path
```

## 1. Route Asset 布局

建议：

```text
runtime/maps/<map_id>/versions/<map_version_id>/routes/<route_id>/<revision>/
  route.yaml
  route.csv
  policy.yaml
  feasibility_report.json
  preview.geojson
  tuning.yaml              # 可选，若经过人工/规则微调
```

Route Asset 不允许脱离 map version 单独成为匿名 CSV。Route revision 是绑定到 READY map 的独立子资产；新增 route revision 不修改已冻结的 map manifest。

## 2. route.yaml

READY revision 最低字段：

```yaml
schema_version: 1
route_id: greenhouse_main_route
revision: 1
frame_id: map
map_binding:
  map_id: greenhouse_a
  map_version_id: map_20260810_120000_1234abcd
  manifest_sha256: sha256:<64 lowercase hex>
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

`semantic_map.geojson` 与 `coverage.yaml` 必须都是 READY map manifest 已冻结的 canonical assets。派生器不得接受位于同一路径但内容 hash 已变化的语义文件。

任何 map/semantic/coverage/vehicle/policy hash 变化都必须使旧 Route Asset 失效或重新派生，禁止只靠文件名判断兼容。

Route 建立时先写 `DRAFT`；feasibility report 与 preview 必须先生成完成，最后一次写 `route.yaml` 才能晋升为 `READY`。READY revision 之后不得原地重新生成 preview、修改 CSV 或覆盖验收报告。

## 3. Route Policy

`policy.yaml` 描述“如何从语义地图派生路线”，不复制 Vehicle Profile 的几何真值。建议字段：

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

其中 footprint、vehicle width、wheelbase、minimum turning radius 等必须从 `profiles/platforms/<platform>.yaml` 读取/校验，而不是在 policy 中维护第二份。

`footprint_check_resolution_m` 是正式 policy 字段；V25-09A 首版 feasibility backend 复用现有 `agt_coverage_planning.path_validator` 的内部自适应采样策略，因此该字段先作为未来 validator/backend 可配置采样的冻结合同，不得伪称当前已经覆盖 backend 的每个内部采样步长。

## 4. Semantic Map 派生规则

第一版允许使用已有语义：

- `row_centerline`：按 `row_interpretation` 解释为道路或作物行中心线
- `access_lane`：显式通行中心线
- `headland_zone`：允许调头/连接的区域约束
- `keepout_zone` / `exclusion_zone`：不可通行
- `entry_pose`：候选起终点/入口
- `waypoint`：命名锚点，可作为任务停顿、定位锚点或路线控制点，但其存在不自动产生执行顺序

派生器必须只读 Semantic Map；生成 Route Asset 不得反向改写 GeoJSON。

### V25-09A 首版派生 backend

首版 `semantic_boustrophedon_mvp` 支持 `planning_mode=annotated_rows`：

- `direct_swaths`：enabled `row_centerline` 直接作为道路，可按 policy 加入 `access_lane`
- `crop_centerlines`：复用既有确定性规则，将相邻作物行中心线派生为行间道路；派生对象使用 `preview_aisle_xxx` 引用，并由冻结 Semantic Map + Route Policy 决定性复现
- 道路按 `work_direction` 法向排序并交替方向形成 boustrophedon 顺序
- 首版道路间 connector 仅生成 straight candidate，不声称已经实现 Hybrid-A*/Reeds-Shepp 连接

straight candidate 是否真的可走必须由后续完整 footprint / kinematic gate 判定。对于不能原地转向的车型，若连接段产生不满足最小转弯半径的姿态变化，应得到 `INVALID`，后续再由 Hybrid-A*、State Lattice 或 Reeds-Shepp connector backend 替换，而不改变 Route Asset schema。

## 5. route.csv

Route CSV 是高密度几何与运行提示，不是唯一真源。至少：

```csv
seq,segment_id,x,y,yaw,direction,v_ref,curvature,clearance,semantic_ref,event_ref
0,s000,1.000,2.000,0.000,F,0.30,0.000,1.20,row_01,
1,s000,1.050,2.000,0.000,F,0.30,0.000,1.18,row_01,
2,s001,1.100,2.000,3.142,R,0.15,-0.30,0.90,headland_01,tomato_stop_003
```

约束：

- `frame_id` 固定由 `route.yaml` 声明，当前正式 Route Asset 使用 `map`
- `direction` 只允许 `F`/`R`
- 方向变化前后必须有 segment boundary，并由 policy 决定是否要求停车
- `v_ref` 是路线建议值，不绕过 controller/safety 的速度上限
- `semantic_ref` 可以引用源 Feature，也可以引用由冻结语义规则确定性产生的 `preview_aisle_xxx`；匿名未知 ID 必须拒绝
- `event_ref` 只引用任务/语义锚点；Route Executor 不自行执行采摘、喷药、拍照等业务

首版派生器只生成 `F` segment；`R` 已作为正式 Route Asset 合同冻结，真正的 reverse-aware connector/planner backend 后续实现。

## 6. Vehicle Feasibility

Route 进入 READY 前必须做完整 footprint sweep，不允许只检查中心线。

输入：

```text
Global Navigation Map
+ semantic field / keepout / exclusion
+ Route samples
+ canonical Vehicle Profile navigation_footprint
+ kinematic limits
```

至少检查：

- footprint collision count
- minimum clearance
- unknown-cell intersections
- curvature / minimum-turning-radius violations
- reverse permission
- direction-change feasibility
- full footprint 是否保持在 enabled `field_boundary` 且不进入 `keepout_zone` / `exclusion_zone`
- semantic reference validity
- entry/exit feasibility
- task stop pose footprint feasibility（当 route 含相应 event/stop 时）

首版代码直接复用 `agt_coverage_planning.path_validator` 的 full-footprint、OccupancyGrid、unknown-space、clearance 与 minimum-turning-radius 几何核心；额外叠加 Semantic Map 的 field/exclusion/keepout footprint gate，避免 coverage route 和普通 Route Asset 使用两套不一致的车辆几何判断。

输出 `feasibility_report.json`，例如：

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

任何 ERROR 级碰撞、语义可通行域或运动学违规都不能 READY。

## 7. 可视化合同

`preview.geojson` 只用于离线 UI/RViz/Qt/Web 预览，不是执行真值，但它是 READY Route 的冻结验收证据之一。建议图层：

```text
base occupancy reference
semantic rows / lanes / headlands / keepout
route centerline
forward/reverse segment style
sampled vehicle footprint polygons
clearance hot spots
stop/event anchors
invalid footprint samples
```

可视化必须使用与规划相同的 canonical `navigation_footprint`，不能为了显示方便用简化小车尺寸后声称可通行。首版 `preview.geojson` 至少输出 route segment、采样 footprint、event anchor，以及 Occupancy/kinematic 或 semantic-free-space 的 invalid footprint。

READY revision 的 preview 不允许原地重新生成。显示端应读取现有 preview；若正式预览证据的采样策略或几何需要变化，则产生新的 route revision 并重新验收。

## 8. 微调合同

允许离线人工/规则微调，但必须 non-destructive。`tuning.yaml` 保存相对 base route 的 patch/参数：

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

V25-09A 首版实现 `lateral_offset` 与 `speed_scale`。任何几何微调后必须：

```text
new revision -> 重新采样/读取 -> 重新 footprint sweep
             -> 重新 semantic gate -> 重新 feasibility/preview -> 新 hash
```

禁止在 GUI 中拖动 CSV 点后直接覆盖 READY route。

## 9. Planner baseline 与创新边界

Route policy 可以调用 Hybrid-A*、State Lattice、Fields2Cover/Reeds-Shepp 或自研 planner。统一 Route Asset/feasibility lineage 与 planner backend 解耦。

研究创新可以放在：

- semantic corridor / row/headland 约束
- clearance-aware objective
- task-stop aware routing
- forward/reverse penalty 与方向切换
- 不同车型可行性比较
- 多时期地图下路线复用/重验证

而不是把“支持倒车/考虑 footprint”本身当作唯一创新。

## 10. Runtime boundary

后续 ROUTE backend 只消费 READY Route Asset，并把当前 active segment 转成 `odom` frame Runtime Path。Route Asset 本身不发布速度、不拥有 TF、不绕过 project Navigation Capability。
