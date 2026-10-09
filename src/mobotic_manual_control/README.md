# mobotic_manual_control

Converts `sensor_msgs/msg/Joy` into a supervised `manual/cmd_vel` command source for the vehicle
supervisor. It also publishes `mobotic_interfaces/msg/ManualControlState` on `manual/state`.

The node never calls drive services. A stale joystick or invalid joystick layout immediately produces
a zero command and an inactive state. The supervisor remains the only component allowed to select this command
source and authorize motion.
No hold-to-drive button is required in the default profile (`button_deadman: -1`).
An optional configured deadman still stops immediately when released. In MANUAL,
stick input can command motion without pressing a button, subject to supervisor
readiness and speed limits; keep sticks neutral during startup and mode changes.

Joystick timeout/disconnection clears the stored velocity, deadman/turbo flags,
and acceleration timing. The callback also detects a receipt gap before accepting
a reconnect message, so timer ordering cannot revive an old command. A neutral
joystick remains at zero after reconnection; nonzero input starts accelerating
from zero on subsequent fresh messages. A backward clock jump also clears the
command and requires fresh joystick input. No deadman release/repress sequence
is imposed by this reset.

The temporary USB profile is for Logitech F310/F710, using the hardware XInput
setting and `joy/game_controller_node`. It uses SDL's standardized ROS layout,
not the manufacturer-dependent ordering of `joy_node`. All four face buttons
(indices 0..3) are unused. Verify the actual input with `ros2 topic echo
/moboterra/joy` on a secured platform before operating.
Its `frame_id` must exactly match the supervisor's `output_frame_id` (both default
to `base_link`); mismatched/empty velocity frames are rejected by the supervisor.

Bringup starts a separate `joystick` node (`game_controller_node`) to publish `joy`.
`mobotic_manual_control` implements the teleop role from the original diagram;
it is not a replacement name for the joystick driver. The supervisor subscribes
to manual command/state, not directly to `joy`.

## Temporary Logitech controls

| Physical control | ROS index | Role |
|:--|:--|:--|
| Left stick horizontal | axis 0 | Steering while moving; yaw reverses with travel direction |
| Right stick vertical | axis 3 | Forward/backward translation |
| Right stick horizontal | axis 2 | Left/right crab translation, no yaw |
| L1 / left shoulder | button 9 | Boost, temporary B1 role |
| R1 / right shoulder | button 10 | Same boost; either shoulder works |
| Back | button 4 | Toggle MANUAL (0) / AUTO_VELOCITY (1), temporary RP1 role |
| Left stick vertical, face buttons, Start, stick clicks, D-pad and triggers | unused | No assigned function |

The two right-stick translation axes are independent. Left-stick yaw is proportional to
forward/reverse input, so it does not command stationary rotation. The supervisor
still applies planar speed and safety limits. L1 or R1 boosts while held; pressing
both does not stack the boost. Releasing them returns to the normal scale without
disabling driving. Boost cannot bypass supervisor limits, and USB controls do not
replace the independent RP2/STOP safety chain.

`manual/state.deadman_pressed` reports an actual configured button, not simulated
permission; it is false with the default disabled deadman. The supervisor requires
fresh `connected` and `command_active` flags. The manual node includes any optional
deadman requirement in `command_active`.

Standardized indices are documented by the
[ROS joystick driver](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md).
The F310's input-mode switch is described in
[Logitech's guide](https://www.logitech.com/assets/47879/f310-gamepad-quick-start-guide.pdf).
Keep the gamepad's stick/D-pad MODE setting in normal stick operation and verify
axes on the target Linux machine. Do not use this YAML with raw `joy_node` ordering.

## Mode selection

Back's rising edge publishes `VehicleModeRequest` on `vehicle/mode_request`,
independently of boost. It reads fresh `vehicle/mode_state`: MANUAL requests
AUTO_VELOCITY, while AUTO_VELOCITY or AUTO_DIRECT requests MANUAL. It never requests
AUTO_DIRECT. No local toggle state is assumed after terminal requests.
During a reported transition, or without fresh supervisor mode feedback, presses
are ignored; release and press again after readiness. Holding does not repeat.
Startup, timeout, invalid layout or reconnect primes the input, so an already-held
button cannot change mode. Fresh, nonzero, ordered Joy timestamps are required
for mode requests. Missing optional mode inputs do not break manual driving.

`button_mode_switch: -1` disables joystick mode requests. `mode_switch_type:
position` prepares a future two-position RP1 input; verify the receiver index
and Manual/Auto polarity before setting it. This is not a REMdevice CANopen
implementation. `MoboTerra_documentation.docx` in the parent folder defines RP1,
B1, B2, RP2 and STOP roles but does not provide their receiver TPDO/Joy mapping.

The supervisor stops the platform through the outgoing command path, verifies
0.25 s measured standstill, then selects exactly one mode and waits for a new
command. Requests use the same logic as terminal `vehicle/set_mode`; a request
is not confirmation that the transition completed. Observe `vehicle/mode_state`.
AUTO_DIRECT (2) is accepted only by `vehicle/set_mode`, not the mode-request topic:

```bash
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode "{requested_mode: 2}"
```

This separates supported control APIs; it is not ROS client authentication.
The physical USB mapping and stop behavior still require hardware validation.

The REMdevice CANopen receiver and B2 fault-reset control are not implemented.
RP2/STOP remain in the independent safety chain. Global error clearing remains available
through the driver's namespace-relative `clear_errors` service.

ROS-free regression tests:

```powershell
python -m unittest discover -s src/mobotic_manual_control/test -v
```
