---
description: "Use when editing C++ sources, headers, or CMake for cloisim_rviz_plugin. Captures local conventions for RViz panel code, Qt usage, and plugin packaging."
applyTo: "{include/**/*.hpp,src/**/*.cpp,CMakeLists.txt}"
---

# cloisim_rviz_plugin C++ Instructions

## Scope

Apply these rules when changing:
- `include/**/*.hpp`
- `src/**/*.cpp`
- `CMakeLists.txt`

This package is a ROS 2 RViz plugin built as a shared library with Qt5 Widgets, `pluginlib`, `rviz_common`, and `rclcpp`.

## Build and Language Constraints

- Keep code compatible with **C++17**.
- Preserve existing warning expectations from `CMakeLists.txt`: `-Wall -Wextra -Wpedantic`.
- Keep the package buildable as a plugin library, not a standalone executable.
- When adding new source files, update `CMakeLists.txt` for:
  - source list
  - MOC header list when `Q_OBJECT` is used
  - install/export wiring if public headers are added

## Header and Source Layout

- Put public panel declarations in `include/panels/`.
- Put implementations in `src/panels/`.
- Keep one primary class per panel pair: `foo_panel.hpp` and `foo_panel.cpp`.
- Keep plugin export in the corresponding `.cpp` file using `PLUGINLIB_EXPORT_CLASS(...)`.

## Namespace and Class Conventions

- Use namespace `cloisim_rviz_plugin`.
- RViz panels should inherit from `rviz_common::Panel`.
- Keep class names descriptive and panel-oriented, for example `JogControlPanel`.
- Prefer explicit constructors taking `QWidget * parent = nullptr`.

## Qt and RViz Usage

- Use Qt Widgets already present in the package; do not introduce QML.
- Keep UI construction local to the panel unless there is a repeated reusable widget.
- Use signal-slot connections with typed function pointers or lambdas.
- Avoid UI work outside the Qt/RViz object lifecycle.
- `onInitialize()` is the right place to acquire the RViz ROS node abstraction and create ROS interfaces.

## ROS 2 Integration Rules

- Create subscriptions and publishers from the RViz raw node acquired through `getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()`.
- Match QoS to the topic semantics instead of using one blanket profile everywhere.
- When reconfiguring publishers/subscriptions, reset the old handle before replacing it.
- Keep topic names user-editable when that is already part of the panel behavior.

## Threading and Lifetime

- Be skeptical of raw background threads in panel code. Prefer timers or clearly owned worker logic unless a thread is unavoidable.
- If a thread exists, ensure deterministic stop/join behavior on destruction.
- Do not manually delete child widgets that are already owned by Qt parent-child ownership unless there is a specific proven need.
- Avoid capturing invalid panel state from long-lived lambdas or threads.

## XML and Robot Description Handling

- Parse robot description defensively.
- Reject malformed XML early and log a clear ROS error.
- Treat missing URDF fields as expected runtime cases, not fatal crashes.
- Keep joint range handling synchronized with displayed command widgets.

## CMake Rules

- Keep `CMAKE_AUTOUIC`/`CMAKE_AUTORCC` as-is unless there is a concrete reason to change them.
- If a header contains `Q_OBJECT`, ensure it is included in the MOC header list.
- Keep `pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)` intact.
- Preserve install rules for headers and the shared library.

## Avoid

- Do not add unrelated refactors while fixing one panel issue.
- Do not convert the package to Qt6 unless the whole ROS/RViz dependency stack requires it.
- Do not bypass `pluginlib` registration.
- Do not assume `.history/` files are source of truth.
