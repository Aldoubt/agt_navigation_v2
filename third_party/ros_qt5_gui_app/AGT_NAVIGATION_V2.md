# AGT Navigation V2 integration

This source tree is maintained on the `agt-navigation-v2` branch of
`Aldoubt/Ros_Qt5_Gui_App`. It retains the original GPL-2.0 license and upstream
history from `chengyangkj/Ros_Qt5_Gui_App`.

The branch is the Qt frontend used by `agt_navigation_v2`. Its project-specific
contract is:

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
- manual velocity remains on the configured project input and must pass through
  `agt_safety` before reaching a chassis driver.

`agt_navigation_v2` vendors a fixed snapshot of this branch under
`third_party/ros_qt5_gui_app`. Builds do not clone the fork at configure time;
the fork exists for review, attribution, and controlled source updates.
