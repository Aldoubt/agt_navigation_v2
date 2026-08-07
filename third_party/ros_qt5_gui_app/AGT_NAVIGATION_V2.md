# AGT Navigation V2 integration

This source tree is maintained on the `agt-navigation-v2` branch of
`Aldoubt/Ros_Qt5_Gui_App`. It retains the original GPL-2.0 license and upstream
history from `chengyangkj/Ros_Qt5_Gui_App`.

The branch is the Qt frontend used by `agt_navigation_v2`. It remains a client
of the project-owned ROS 2 managers and does not own system, mission, map,
experiment, bag, safety, or chassis state. Its project-specific contract is:

- `UiLayoutId=control-center-v1` selects the replaceable operator shell;
  `UiLayoutId=legacy` keeps the existing docking workspace as a fallback;
- `UiThemeId` and `UiDensity` are presentation settings only and never grant a
  capability;
- `UiCapabilityPolicy` is built only from the selected runtime profile, while
  `RobotStateViewModel`, `MissionViewModel`, and `SystemModeViewModel` isolate
  widgets from ROS message and Action details;
- `/agt/system/robot_state` and `/agt/missions/status` are the authoritative UI
  status feeds and use reliable transient-local subscriptions;
- the formal navigation page executes and controls missions through
  `/agt/missions/execute` and `/agt/missions/set_run_state`;
- mapping-session controls call `/agt/mapping/manage_session`; the GUI never
  reproduces the save-grid, stop, PCD wait, offline candidate, or commit chain;
- automatic relocalization calls the project `/agt/localization/relocalize`
  Action rather than publishing a second recovery workflow;
- map and Bag/experiment pages use `/agt/maps/list`, `/agt/maps/manage`,
  `/agt/data/bags/list`, and `/agt/data/bags/manage`; returned manager asset
  paths are displayed or passed through without path reconstruction;
- system mode changes call `/agt/system/change_mode`; the GUI never starts a
  launch file itself;
- navigation mode arguments come only from the active map summary carried by
  `RobotState`; an incomplete active-map context blocks the request locally and
  remains subject to the backend validation;
- direct `/goal_pose` publication is an advanced debug capability and is
  disabled unless `EnableDebugGoalPose=true`;
- the old waypoint execution UI is compatibility-only and requires both
  `EnableTaskExecution=true` and `EnableLegacyWaypointExecution=true`;
- task chains execute through
  `/agt/navigation/execute_waypoint_task` (`agt_interfaces/ExecuteWaypointTask`);
- Nav2 Action feedback/result, not pose-distance polling, determines task state;
- repeated execution is finite and cancellation is forwarded to the Action;
- `EnableTaskExecution=false` disables task controls and is enforced again by
  the ROS2 channel for the mapping profile;
- a missing `EnableTaskExecution` key is fail-closed; only explicit `true`
  enables navigation task dispatch;
- malformed Nav2 map YAML is rejected without terminating the GUI;
- changing a map clears stale topology and rejects sidecar points outside the
  selected map;
- navigation points use two clicks: position first and heading second, with
  right-click/Escape/tool changes canceling incomplete placement;
- `EnableCostmapDisplay` is fail-closed so multi-million-cell costmaps are not
  repeatedly expanded and rendered unless an operator explicitly enables them;
- decorative topology animation is limited to 10 FPS;
- map clicks populate the explicitly selected task row, and topology changes
  refresh every task selector;
- the Task Library topology selector snapshots the selected topology point
  name, metric map pose, and heading into an independent task waypoint; later
  topology edits do not silently rewrite saved task geometry;
- offline map inspection uses view-level pan/zoom and manual navigation cancels
  robot-follow mode; the native window frame remains resizable;
- `UiLanguage` selects persisted `zh_CN` (default) or `en_US` operator text and
  applies after restart; the settings pages use the same explicit bilingual
  strings rather than relying on an absent Qt translation catalog;
- `EnableOfflinePlanningPreview` exposes a preview-only request while the
  offline profile keeps `EnableTaskExecution=false`; ROS Action dispatch is
  queued off the GUI thread;
- manual velocity remains on the configured project input and must pass through
  `agt_safety` before reaching a chassis driver.

The corresponding profile capabilities are independent of presentation:
`EnableMappingSessionControl`, `EnableRelocalization`, `EnableMapManager`, and
`EnableBagManager` default to false in the channel. A page being visible does
not grant any of these operations.

The light and dark themes live under `resources/themes/<theme-id>/`. Each theme
contains a manifest with named tokens and a QSS template. New operator-facing
text must use the existing `UiLanguage::Text(zh_CN, en_US)` boundary.

`agt_navigation_v2` vendors a fixed snapshot of this branch under
`third_party/ros_qt5_gui_app`. Builds do not clone the fork at configure time;
the fork exists for review, attribution, and controlled source updates.
