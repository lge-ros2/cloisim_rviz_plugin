---
description: "Use when adding or editing RViz panel classes in cloisim_rviz_plugin. Focuses on panel lifecycle, topic wiring, UI state, and safe interaction patterns."
applyTo: "{include/panels/**/*.hpp,src/panels/**/*.cpp}"
---

# RViz Panel Instructions

## Scope

These rules apply to RViz panel code under:
- `include/panels/**`
- `src/panels/**`

## Panel Lifecycle

- Construct widgets and layouts in a dedicated helper such as `initializeLayout()`.
- Acquire the RViz ROS node and create ROS interfaces in `onInitialize()`.
- Keep destruction predictable:
  - stop active motion or background work first
  - release ROS interfaces
  - avoid double ownership bugs with Qt widgets

## UI State Rules

- The panel UI is stateful. Treat widget values as the live source of intent only when they are explicitly user-editable.
- For read-only state displays, update from subscribed ROS messages only.
- If a panel rebuilds rows dynamically, keep the widget map keyed by stable semantic identifiers such as joint names.
- When a new robot description changes limits or available joints, update validators and sliders consistently.

## Topic Configuration

- Keep topic fields explicit and visible when operators may need to retarget them.
- Recreate subscriptions/publishers after topic changes instead of mutating hidden internal state.
- Use transient local QoS only where latched-style behavior is actually needed, such as robot description snapshots.
- Do not assume namespaces; read them from the RViz node context and display them clearly.

## Motion/Command Behavior

- Guard command publication against invalid or incomplete panel state.
- If generating a motion plan from current and target joint states, validate:
  - non-empty joint set
  - positive control frequency
  - positive duration
  - finite numeric values
  - ranges consistent with URDF limits
- Stop behavior must leave the panel in a known state and re-enable blocked controls.

## Logging and Operator Feedback

- Use ROS logging for runtime and parse failures.
- Use Qt dialogs sparingly, only for operator-facing validation failures that need immediate attention.
- Prefer clear error messages over silent no-ops.

## Plugin Packaging

- Every panel class intended for RViz must remain exported through `PLUGINLIB_EXPORT_CLASS`.
- If you add a new panel, also update:
  - `plugin_description.xml`
  - `CMakeLists.txt`
  - installed headers/source lists as needed

## Narrow Fix Discipline

- Start from one concrete symptom: build break, plugin load failure, broken widget behavior, bad topic wiring, or incorrect command output.
- Change the smallest controlling code path first.
- Validate with the narrowest available check before broadening scope.

## Avoid

- Do not mix UI cleanup with behavior changes unless the bug requires it.
- Do not introduce hidden topic remapping rules.
- Do not rely on busy-wait loops where an event-driven or timer-based approach would work.
- Do not manually delete Qt-owned child widgets just to mirror raw-pointer members.
