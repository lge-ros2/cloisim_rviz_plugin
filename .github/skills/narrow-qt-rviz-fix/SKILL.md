---
name: narrow-qt-rviz-fix
description: "Debug and fix a narrow RViz/Qt panel issue in cloisim_rviz_plugin. Use when chasing plugin load failures, widget-state bugs, bad topic wiring, thread/lifetime issues, or broken command publication."
argument-hint: "Describe the concrete panel symptom, build error, runtime warning, or operator-visible wrong behavior."
---

# Narrow RViz/Qt Fix

Use this workflow for a small, evidence-driven fix in `cloisim_rviz_plugin`.

## When to Use

- RViz panel fails to load
- `pluginlib` registration looks broken
- a widget stops updating
- a topic publisher/subscriber is wired incorrectly
- a validator, slider, or command field behaves incorrectly
- a panel thread or shutdown path hangs or crashes
- a build error is confined to panel/CMake code

## Expected Outcome

You should finish with:

- one concrete symptom
- one local hypothesis about the controlling code path
- one minimal edit
- one focused validation result

## Procedure

### 1. Start from one anchor

Use one of these as the starting point:
- build error text
- RViz plugin load failure
- one broken widget interaction
- one incorrect topic or message flow
- one crash/hang during panel init or shutdown

Do not explore the whole package first.

### 2. Find the controlling code path

Prefer the nearest owner:
- `plugin_description.xml` for discovery/registration failures
- `CMakeLists.txt` for MOC/build/export failures
- `include/panels/*.hpp` and `src/panels/*.cpp` for panel behavior
- `onInitialize()` for ROS interface setup
- destruction/stop logic for lifetime bugs

If the first file only forwards behavior, move one hop to the file that actually decides it.

### 3. Form one falsifiable hypothesis

Examples:
- the panel is not exported with the same fully qualified type registered in `plugin_description.xml`
- the header uses `Q_OBJECT` but is missing from the MOC input list
- the panel deletes Qt-owned widgets and corrupts shutdown
- the topic reset path recreates one interface but leaves another stale
- command generation accepts invalid numeric state and publishes garbage

### 4. Make the smallest grounded edit

- change the smallest code that controls the observed failure
- do not mix cleanup with the first fix
- preserve public behavior outside the failing slice

### 5. Validate immediately

Prefer the narrowest available check:

1. rebuild the package
2. reload RViz and verify panel discovery/load
3. reproduce the exact widget interaction
4. verify topic publication/subscription behavior

## Local Validation Examples

Because this package lives in the ROS 2 workspace, run commands from:
- `/home/yg/Workspace/cloi_ws`

Typical checks:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select cloisim_rviz_plugin
source install/setup.bash
rviz2
```

For a build-only issue, the first check is enough. For panel behavior, use RViz and reproduce the exact failure.

## Common Failure Modes

- missing `PLUGINLIB_EXPORT_CLASS`
- type mismatch between exported class and `plugin_description.xml`
- `Q_OBJECT` header omitted from the MOC header list
- unsafe manual deletion of Qt child widgets
- stale ROS interfaces after topic edits
- background thread not stopped before destruction
- invalid numeric parsing from `QLineEdit`

## Guardrails

- Keep fixes local to the panel, build file, or plugin manifest involved.
- Prefer ROS logging and explicit operator feedback over silent fallback behavior.
- Do not invent a new architecture for a narrow bug.
- If the first hypothesis fails, move only one nearby hop and retest.

## Completion Checklist

- [ ] I started from one concrete RViz/Qt symptom
- [ ] I identified the nearest controlling file/path
- [ ] I stated one falsifiable hypothesis
- [ ] I made one minimal fix
- [ ] I validated with a focused build or runtime repro
- [ ] I avoided unrelated refactors
