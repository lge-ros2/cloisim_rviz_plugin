# CLOiSim rviz plugins

This is a rivz2 plugins for cloisim.

## Panel

### Jog Control Panel

Command joint command

- subscribe `/robot_description`
- subscribe `/joint_states`
- publish `/joint_command`

## How to run

```shell
colcon build --sym --packages-up-to cloisim_rviz_plugin
. ./install/setup.bash
rviz2
```
