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

### add panel

![image](https://github.com/lge-ros2/cloisim_rviz_plugin/assets/21001946/68516933-0a58-4cdd-a63d-43c84d30f632)

### rviz2 screenshot

![image](https://github.com/lge-ros2/cloisim_rviz_plugin/assets/21001946/9de7d9b1-4377-40f1-a8b4-7b19ca03782e)
