# CLOiSim RViz Plugin

`cloisim_rviz_plugin` is a ROS 2 RViz plugin package for CLOiSim operator panels.

Current package scope:

- one RViz panel plugin: `cloisim_rviz_plugin::JogControlPanel`
- Qt5 Widgets-based panel UI
- ROS 2 topic integration through the RViz node abstraction
- plugin registration through `pluginlib`

## Current Panel

### `JogControlPanel`

The jog panel is an operator-facing joint command panel.

It currently:

- subscribes to `/robot_description`
- subscribes to `/joint_states`
- publishes to `/joint_command`

Practical behavior:

- reads URDF joint limits from `robot_description`
- shows current joint positions from `joint_states`
- lets the operator type or slide target joint values
- publishes one-shot `control_msgs/msg/JointJog` commands
- can generate a simple time-based move sequence from current state to target state

## Package Layout

Key files:

- `include/panels/jog_control_panel.hpp` — panel declaration
- `src/panels/jog_control_panel.cpp` — panel implementation
- `plugin_description.xml` — RViz/pluginlib registration
- `CMakeLists.txt` — shared library build, Qt MOC wiring, install/export rules

Customization files added for this package:

- `.github/instructions/cpp.instructions.md`
- `.github/instructions/rviz-panel.instructions.md`
- `.github/skills/add-rviz-panel/SKILL.md`
- `.github/skills/narrow-qt-rviz-fix/SKILL.md`

## Dependencies

This package depends on:

- ROS 2
- `rviz_common`
- `pluginlib`
- `rclcpp`
- `Qt5::Widgets`
- `sensor_msgs`
- `control_msgs`
- `std_msgs`
- `TinyXML2`

Language/build constraints from the package today:

- C++17
- `-Wall -Wextra -Wpedantic`
- shared-library plugin build, not a standalone app

## Build

Build from the ROS 2 workspace that contains the package:

```shell
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select cloisim_rviz_plugin
source install/setup.bash
```

The old README command used `--sym`. That is wrong here.

## Run

After building:

```shell
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
rviz2
```

## Add the Panel in RViz

1. Open `rviz2`
2. Open the panels menu
3. Add the `JogControlPanel` plugin from `cloisim_rviz_plugin`

![Add panel](https://github.com/lge-ros2/cloisim_rviz_plugin/assets/21001946/68516933-0a58-4cdd-a63d-43c84d30f632)

## RViz Screenshot

![RViz screenshot](https://github.com/lge-ros2/cloisim_rviz_plugin/assets/21001946/9de7d9b1-4377-40f1-a8b4-7b19ca03782e)

## Development Notes

- If a panel header uses `Q_OBJECT`, it must stay in the MOC header list in `CMakeLists.txt`.
- New panels must be registered in `plugin_description.xml`.
- RViz ROS publishers/subscriptions should be created in `onInitialize()`.
- Be careful with Qt ownership and shutdown paths; avoid unnecessary manual deletion of Qt-owned child widgets.

## Typical Failure Modes

- plugin builds but does not appear in RViz because `plugin_description.xml` is incomplete
- panel class uses `Q_OBJECT` but the header is missing from MOC input
- exported class name does not match the registered plugin type
- topic wiring is stale after panel field changes
- panel shutdown misbehaves because of thread/lifetime mistakes
