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

Route Asset 不允许脱离 map version 单独成为匿名 CSV。

## 2. route.yaml

最低字段：

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
  path: ../../semantic/semantic_map.geojson
  sha256: sha256:<64 lowercase hex>
vehicle_binding:
  platform_id: bunker
  platform_profile_sha256: sha256:<64 lowercase hex>
policy_binding:
  path: policy.yaml
  sha256: sha256:<64 lowercase hex>
route_csv_sha256: sha256:<64 lowercase hex>
feasibility_report_sha256: sha256:<64 lowercase hex>
status: READY
```

任何 map/semantic/vehicle/policy hash 变化都必须使旧 Route Asset 失效或重新验证，禁止只靠文件名判断兼容。

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

## 4. Semantic Map 派生规则

第一版允许使用已有语义：

- `row_centerline`：按 `row_interpretation` 解释为道路或作物行中心线
- `access_lane`：显式通行中心线
- `headland_zone`：允许调头/连接的区域约束
- `keepout_zone` / `exclusion_zone`：不可通行
- `entry_pose`：候选起终点/入口
- `waypoint`：命名锚点，可作为任务停顿、定位锚点或路线控制点，但其存在不自动产生执行顺序

派生器必须只读 Semantic Map；生成 Route Asset 不得反向改写 GeoJSON。

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
- `event_ref` 只引用任务/语义锚点；Route Executor 不自行执行采摘、喷药、拍照等业务

## 6. Vehicle Feasibility

Route 进入 READY 前必须做完整 footprint sweep，不允许只检查中心线。

输入：

```text
Global Navigation Map
+ semantic keepout/exclusion
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
- semantic corridor containment
- entry/exit feasibility
- task stop pose footprint feasibility

输出 `feasibility_report.json`，例如：

```json
{
  "status": "PASS",
  "route_id": "greenhouse_main_route",
  "revision": 1,
  "metrics": {
    "length_m": 42.1,
    "reverse_length_m": 3.2,
    "min_clearance_m": 0.41,
    "direction_changes": 2,
    "footprint_collision_count": 0,
    "curvature_violation_count": 0,
    "unknown_intersection_count": 0
  },
  "errors": [],
  "warnings": []
}
```

任何 ERROR 级碰撞/运动学违规都不能 READY。

## 7. 可视化合同

`preview.geojson` 只用于离线 UI/RViz/Qt/Web 预览，不是执行真值。建议图层：

```text
base occupancy reference
semantic rows / lanes / headlands / keepout
route centerline
forward/reverse segment style
sampled vehicle footprint polygons
clearance hot spots
stop/event anchors
invalid samples
```

可视化必须使用与规划相同的 canonical `navigation_footprint`，不能为了显示方便用简化小车尺寸后声称可通行。

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

任何几何微调后必须：

```text
重新采样 -> 重新 footprint sweep -> 重新 feasibility report -> 新 revision/hash
```

禁止在 GUI 中拖动 CSV 点后直接覆盖 READY route。

## 9. Planner baseline 与创新边界

Route policy 可以调用 Hybrid-A*、State Lattice、Fields2Cover/Reeds-Shepp 或自研 planner。算法实现不是本合同的一部分；合同只要求输出满足统一 Route Asset/feasibility lineage。

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
