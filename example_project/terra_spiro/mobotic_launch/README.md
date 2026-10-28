# mobotic_launch

This folder contains launch files for Terraspiro platforms.

## Running the stack

The `ROBOT_NAME` environment value must be set:

```
export ROBOT_NAME=terraspiro
```

To launch the terraspiro stack use the following command:

```
ros2 launch mobotic_launch terraspiro.launch.py stack_type:=real can_interface:=can0
```

You can launch the virtual version of the stack by using the following command:

```
ros2 launch mobotic_launch terraspiro.launch.py stack_type:=virtual can_interface:=vcan0
```

Even on the virtual stack, you can select `can0` as the interface if you have it configured and running.

## Stop before turns

`circle_path_publisher.py` bypasses stop-before-turn behavior by default. To
enable a fixed stop at the configured turn nodes, start it with:

```
ros2 run mobotic_launch circle_path_publisher.py --ros-args \
  -r __ns:=/mobotic -p stop_before_turn_enabled:=true
```

The `stop_before_turn_enabled` parameter defaults to `false`. When it is false,
the next loop goal starts immediately at configured stop nodes. When it is
true, `stop_duration_sec` controls the fixed dwell. This path behavior is
independent of battery SOC and of the charge-rate controller.
