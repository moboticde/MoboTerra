# MoboTerra interface source audit

The initial inspection on 19 October 2026 found an older checkout and the
updated customer interface in local stash `a87fcd5`, including its untracked
parent. At the user's request, that implementation has now been integrated
into the active source tree. The stash remains intact. Deployment conflicts
were resolved while retaining the newer installer, GUI/CAD images and smoke checks.
The CAN operator implementation and capture workflow are described in the
[PAIL commissioning guide](remdevice_commissioning.md).

## What was checked

The audit read implementation files, not just their README descriptions:

- `src/mobotic_interfaces/msg/OperatorState.msg`, `VehicleModeState.msg`,
  `DriveStatus.msg`, `WheelModuleStatus.msg`, `WheelModuleCommandArray.msg`,
  `WheelModuleCommand.msg`, `ScannerSafetyState.msg` and `srv/SetVehicleMode.srv`.
- `src/mobotic_supervisor/mobotic_supervisor/supervisor.py`: topic construction,
  mode requests, effective state precedence, source selection, final command
  authority, required fault collection and reset callbacks.
- `src/mobotic_manual_control/mobotic_manual_control/manual_control.py`:
  operator publication, input known flags, neutral/deadman rearm and raw `joy`.
- `src/mobotic_driver/src/mobotic_driver.cpp`: final-command subscriptions,
  internal services, per-drive current/status, PDO acquisition and physical
  position/enable/publication gates.
- `src/mobotic_odometry/mobotic_odometry/odometry_node.py`: renamed measured
  velocity, odometry and reset endpoints.
- `src/mobotic_safety/mobotic_safety/scanner_safety.py`: generic scanner field
  status and per-scanner diagnostics.
- `src/mobotic_bringup/launch/moboterra.launch.py`, shared configuration,
  deployment Compose and installers in both code states.

The initial documentation audit used separate QA copies. The subsequent source
integration restored the tracked and required untracked implementation files
and merged the deployment changes into the pulled installer. Python source
regressions now run against the active checkout, including supervisor interface
tests and CAN protocol/provider tests with clock/message doubles. These checks
do not establish a Jazzy/C++ build, a live rqt graph or hardware commissioning.
Validation on the integrated source passed 295 Python source tests across eight
packages and 20 deployment tests under WSL. Shell entry points were normalized
to LF for Linux execution. The available WSL installation has no ROS/Jazzy,
Docker or C++ compiler, so the complete build and virtual smoke run remain open.

## Confirmed interface changes

| Earlier code | Latest locally implemented code |
| --- | --- |
| `manual/cmd_vel`, `manual/state` | `operator/cmd_vel`, `operator/state` |
| `vehicle/mode_request`, `vehicle/mode_state` | `operator/mode_request`, `vehicle/mode` |
| MANUAL, AUTO_VELOCITY, AUTO_DIRECT | MANUAL, AUTONOMOUS, KINEMATICOFF, automatic SAFETY/ERROR, initial UNKNOWN |
| `cmd_vel`, `agv_vel`, `odometry`, `odometry/reset` | `vehicle/cmd_vel`, `vehicle/measured_velocity`, `vehicle/odom`, `vehicle/reset_odometry` |
| Two driver joint-command subscriptions | Supervisor owns one final `wheel_modules/command` motion array |
| Management on `wheel_modules/command` | Management on internal `wheel_modules/operation_command` |
| Driver-owned public `clear_errors` | Supervisor transaction at `wheel_modules/clear_errors` |
| Flat module current/status | Separate steering/traction DriveStatus with amperes and known flags |
| Separate description launch | Integrated `start_description=true` default |
| ARM64 and baked YAML in pulled deployment | Saved update adds AMD64 and mounted configuration/state; merge remains necessary |

These names are relative to the robot namespace, normally `/moboterra`.
Numeric aliases AUTO_VELOCITY=1 and AUTO_DIRECT=2 remain in the updated message
definitions, but the public mode names are AUTONOMOUS and KINEMATICOFF.

## Release constraints that must remain visible

The saved driver currently requires `virtual_position_source` for enable and
authoritative joint publication. Only virtual launch sets it true. Physical
traction reference/scale/identity recovery has not been commissioned and the
durable helper is not a verified hardware recovery backend. Reading Elmo's
position at power-on is implemented; exact recovery after unpowered wheel
movement still requires secured validation through Elmo. Direct encoder access
is not part of the implementation.

REMdevice input requires a verified receiver mapping. USB gamepads explicitly
publish `estop_state_known=false`; unknown is not a confirmed released E-stop.
Safety enable and independent STO mappings remain unknown where unconfigured.

The source/deployment merge is complete, including the required saved untracked
files. Jazzy integration/virtual checks and secured hardware acceptance remain
required. Source integration is not evidence that the robot's installed image
or host-mounted profile has been upgraded.

## Check the installed version

In a sourced container shell:

```bash
ros2 interface show mobotic_interfaces/msg/OperatorState
ros2 interface show mobotic_interfaces/msg/VehicleModeState
ros2 interface show mobotic_interfaces/msg/WheelModuleCommandArray
ros2 topic list -t
ros2 service list -t
```

Use the [command reference](commands.md) and [ROS interface manual](ros_interface_manual.md)
for the updated endpoints. Missing definitions mean the wrong image/source was built;
topic absence alone may instead mean its provider is not running.
