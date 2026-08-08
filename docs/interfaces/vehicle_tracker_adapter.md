# Vehicle Tracker Adapter Contract

本合同定义 Route/Runtime Path 与具体车型控制器之间的适配边界。它的目标是让同一 Navigation Capability 可以适配不同底盘，而不把车型几何、控制器细节或任务业务泄漏到 Mission/Route schema。

## 1. Boundary

```text
READY Route Asset
   -> active Route Segment
   -> Runtime Path (odom)
   -> Vehicle Tracker Adapter
   -> Controller Backend
   -> project safety chain
   -> chassis
```

Vehicle Tracker Adapter 不负责：

- global planning
- map alignment / relocalization
- map -> odom TF ownership
- Mission/BT task execution
- harvest/spray/capture business actions
- final safety override

## 2. Canonical inputs

```text
Runtime Path
Current odometry / pose
Vehicle Platform Profile
Tracker tuning profile
Local obstacle/cost evidence
```

`profiles/platforms/<platform>.yaml` 是 footprint、wheelbase/track、turning limits 和 platform policy 的 canonical source。Tracker tuning 只允许保存 controller/tracker 参数，例如 lookahead、gain、sampling horizon、speed scaling；不得维护第二份 vehicle geometry truth。

## 3. Canonical outputs / feedback semantics

适配层应向上提供统一状态语义：

```text
state
active_segment_id
active_path_index
tracking_direction
cross_track_error
heading_error
remaining_distance
command_valid
failure_code
```

具体底盘 backend 可以不同，但 Mission/Navigation Capability 不应依赖某个 controller plugin 的私有状态。

## 4. Direction and stop transitions

支持倒车的 Route Asset 使用 `F` / `R` segment。方向切换必须发生在 segment boundary；如果 policy 要求 `direction_change_requires_stop=true`，Tracker Adapter 必须先达到停车条件后才能开始反向 segment。

不允许通过把负速度隐藏在同一连续 path 中绕过方向切换状态。

## 5. Feasibility vs tracking

离线 Route Feasibility 证明“在静态地图、车型 footprint 和运动学约束下，这条路线可通行”。Tracker runtime 证明“当前实际状态能够跟踪该路线”。两者不得混为一谈：

```text
Offline PASS != runtime success guarantee
Runtime tracking failure != route geometry necessarily invalid
```

因此 tracking error、controller abort、local obstacle stop 等必须作为运行证据单独记录。

## 6. Controller reuse

当前可以复用 Nav2 Controller Server / MPPI / RPP / FollowPath 作为 backend。未来可加入 Ackermann、Pure Pursuit、Stanley、MPC 等 tracker，但它们必须保持相同上层 capability、安全和 chassis command ownership。

## 7. Route compatibility gate

加载 READY Route Asset 时至少验证：

```text
route map binding matches active map
route vehicle profile hash matches selected platform
route feasibility status = PASS
route direction semantics supported by tracker
required local-control readiness is healthy
```

任何不匹配都必须 fail-closed，而不是尝试用当前车型“凑合”跟踪另一车型的路线。
