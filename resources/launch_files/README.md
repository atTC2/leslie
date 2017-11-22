# PIPS Launch Files
The Personal Item Protection System (PIPS) can be launch in simulation or on a real robot.

## For Simulation
Launch PIPS with the following command:

```
roslaunch leslie pips_stage.launch
```

## For a Real Robot
Launch PIPS with the following command:

```
roslaunch leslie pips_robot.launch
```

## Launch Parameters
There are a number of optional arguments, all of which have defaults.

Usage is:

```
roslaunch leslie pips_<sim_or_robot>.launch <arg>:=<value>
```

Options are:

| Argument Name | Description | Default |
| ------------- | ----------- | ------- |
| `sleep_multiplier` | The sleep multiplier to use (scale the delay between launching each item, `0` would load everything instantly and a large number would wait longer before launching each individual component). | `1` |
| `follow_people` | Whether to launch the follow node which turns and tracks a suspected theif upon an alarm being triggered. | `false` |
| `use_depth_for_change_node` | Whether to use the depth camera for change detection. | `false` |
| `rviz` | Whether to display RViz. | `true` |
| `rqt_graph` | Whether to display the RQT Graph. | `false` |
| `rviz_config_file` | The RViz configuration profile to use (found in `/resources/rviz_configs`). | `laser_and_particles.rviz` |
| `map_file` | The map file to use (would require other changes in various places to support the home of the robot as well as table locations). | `$(find leslie)/resources/maps/newlowerground.yaml` |

