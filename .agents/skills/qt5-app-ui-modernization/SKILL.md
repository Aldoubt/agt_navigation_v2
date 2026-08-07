---
name: qt5-app-ui-modernization
description: Use when redesigning, polishing, restructuring, theming, or replacing the Qt5 Widgets frontend used by agt_navigation_v2 and its Ros_Qt5_Gui_App fork, while preserving ROS 2 interfaces, profile capabilities, map/task contracts, safety boundaries, and vendored-fork provenance.
metadata:
  short-description: Modernize the AGT Qt5 robot console safely
---

# AGT Qt5 UI Modernization

Modernize the maintained Qt5 frontend without moving business ownership out of
the project ROS 2 managers or weakening a runtime profile.

## Required Context

1. Read the repository root `AGENTS.md` completely.
2. Read `third_party/README.md`, `src/agt_ui_bridge/README.md`, and the active
   profile JSON under `src/agt_ui_bridge/config/`.
3. Read [AGT UI contracts](references/agt-ui-contracts.md) before changing ROS
   adapters, capabilities, map/task behavior, or fork provenance.
4. Read [design system](references/design-system.md) for theme, layout,
   density, responsive, or operator-text changes.
5. Read [Qt5 Widgets patterns](references/qt5-widgets-patterns.md) for shell,
   page, ViewModel, message-bus, QSS, resource, or thread changes.

## Workflow

1. Fetch the main repository and Qt fork remotes. Record the actual
   `origin/main` and `origin/agt-navigation-v2` SHAs before editing.
2. Confirm that `third_party/ros_qt5_gui_app/.agt-fork-commit` and the SHA in
   `third_party/README.md` describe the current vendored snapshot.
3. Classify each requested change as theme, layout/shell, page, ViewModel, ROS
   adapter, or profile capability. Keep business FSMs and asset mutation in ROS
   managers.
4. Preserve existing map `QGraphicsView`, Task Library, task repository, and
   ROS channel implementations. Extend their public boundaries instead of
   recreating them inside widgets.
5. Implement Qt source changes in the maintained fork first. Keep GPL-2.0
   `LICENSE`, attribution, and upstream history intact.
6. Build the fork and verify callback-to-UI crossings use queued signals. Do
   not wait for ROS services or Actions on the GUI thread.
7. Commit and push the fork branch. Synchronize that exact commit into
   `third_party/ros_qt5_gui_app`, update `.agt-fork-commit`, and update the
   pinned SHA and change summary in `third_party/README.md`.
8. Update profile templates and tests in `agt_ui_bridge`. Theme and layout keys
   may change presentation only; capabilities remain profile-owned and
   fail-closed.
9. Run:

   ```bash
   ./tools/build_ros_qt5_gui_app.sh
   python3 -m pytest -q src/agt_ui_bridge/test/test_ros_qt5_gui_profiles.py
   python3 .agents/skills/qt5-app-ui-modernization/scripts/validate_ui_contract.py
   ```

10. Report affected pages, themes, ROS interfaces, fork/pin SHAs, build/test
    results, and any hardware validation that remains outstanding.

## Non-Negotiable Boundaries

- Treat Qt as a client. Never place system mode, Mission, active-map,
  experiment, bag-process, safety, or chassis ownership in a widget/ViewModel.
- Call project Actions and manager services. Do not call Nav2 native Actions,
  launch processes, shell commands, or rosbag subprocesses from Qt.
- Keep manual velocity on `/agt/cmd_vel_manual`. Never publish
  `/agt/safety/cmd_vel` or `/agt/chassis/cmd_vel`.
- Keep `/goal_pose` hidden and disabled by default; it remains a deprecated
  advanced debugging path.
- Never reconstruct active-map paths. Consume map-manager summaries and the
  active-map context in `RobotState`.
- Keep READY raster editing disabled in navigation, offline, and teach
  profiles. Keep candidate open/save-as escape paths disabled.
- Keep offline and teach task execution disabled in both widgets and the ROS
  channel.
- Do not let `UiThemeId`, `UiLayoutId`, `UiDensity`, or diagnostics visibility
  grant a capability.
