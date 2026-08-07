# Qt5 Widgets Patterns

## Separation

Use this direction only:

```text
ROS2 channel -> project message bus -> ViewModel -> queued Qt signal -> page
page -> ViewModel -> project message bus -> ROS2 channel -> project API
```

Pages validate local form shape and render state. ViewModels expose UI-ready
state and commands. The ROS2 channel translates generated ROS messages and
performs a second profile-capability check. Manager nodes retain business FSMs,
asset ownership, process control, and audit logs.

## Threading

- Never call `wait_for_service`, `wait_for_action_server`, or synchronous future
  waits from the GUI thread.
- Queue message-bus callbacks into `QObject` using
  `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` before touching widgets.
- Schedule ROS requests on the ROS executor. Treat unavailable endpoints as
  explicit status, not a UI hang.
- Cancel project Actions through their Action client and show the resulting
  backend status. Do not infer completion from pose, time, or distance.

## Shell And Pages

Keep `UiLayoutManager` limited to shell selection. Keep `UiPageRegistry`
limited to profile-driven information architecture. Reuse existing docking
widgets, map display, Task Library, and diagnostic widgets inside either shell.
Do not clone their data models into new pages.

Page visibility and capability are separate checks. A page may be visible for
read-only inspection while all mutation/execute commands remain disabled.

## QSS And Resources

- Load theme manifests and QSS from `resources/themes/<id>/`.
- Replace named tokens once in `UiThemeManager`; do not construct per-widget
  color styles in page code.
- Keep semantic object names/properties stable for QSS selectors.
- Install/copy resources next to the executable; do not fetch at runtime.
- Use standard Qt icons or existing project resources. Do not draw one-off SVG
  controls when the project already has a suitable icon.

## Map Performance

- Do not recreate `QGraphicsScene` or all map cells on status updates.
- Keep pan/zoom as view transforms; never mutate map coordinates during view
  navigation.
- Preserve two-click position/heading authoring and incomplete-placement
  cancellation.
- Keep full costmap rendering disabled in large-map profiles unless explicitly
  enabled.
- Route map/task mutations through the existing map display and Task Library.

## Advanced Docking System

Reuse Advanced Docking System only as the workspace host and legacy layout
adapter. Do not make dock persistence a source of profile capability, current
page, Mission state, or active-map identity. A restored dock must still obey
the current profile policy.
