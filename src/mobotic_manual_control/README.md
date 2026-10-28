# mobotic_manual_control

The default provider is the CAN `remdevice_control` node, named `operator`, for
PAIL/Ecobox node 9 on can0 at 500 kbit/s. Its physical mapping is unverified, so
it currently captures input and publishes zero motion. No hold-to-drive button
is assigned. Follow the [capture and commissioning guide](../../docs/remdevice_commissioning.md).
Select `operator.source: usb` and `operator.usb_driver: joy_node` for the PAIL's
USB joystick connection. Raw input remains available with
`operator.usb_mapping_verified: false`; verify the USB indices/polarities before
enabling motion. Use `game_controller_node` for the Logitech profile described
below. Launch starts exactly one provider. CAN and USB mappings are independent.

A deadman means hold-to-drive: releasing a configured button outputs zero motion.
This optional feature is implemented for both transports, but remains disabled
by default (`button_deadman: -1` for USB, `deadman: null` in a verified CAN map).

Converts `sensor_msgs/msg/Joy` into a supervised `operator/cmd_vel` command source for the vehicle
supervisor. It also publishes `mobotic_interfaces/msg/OperatorState` on `operator/state`.

The node never calls drive services. A stale joystick or invalid joystick layout immediately produces
a zero command and an inactive state. The supervisor remains the only component allowed to select this command
source and authorize motion.

For CAN, `operator/state.connected` reports fresh physical LINK/EMGY/STOP
evidence. It is independent of command readiness: an EMCY or incomplete PDO pair
sets `command_active: false`, discards motion, and requires fresh neutral rearm.
The physical status retains its own digital acquisition time and expires after
250 ms; EMCYs and publishing timers cannot refresh it. STOP/radio loss still
inhibit immediately. This lets supervised zero-target controller initialization
continue while the motion source rearms, without accepting an old command.
The measured RP2 selector is published separately on `operator/scanner_selector`
with its digital acquisition timestamp and no motion axes. Publishing that
physical switch state never feeds the teleop callback or rearms motion.
Hardware bringup uses a separate receive-only SocketCAN channel filtered to the
operator receiver IDs and CAN errors. This prevents drive-message bursts from
dropping operator samples; all input freshness checks still use CAN acquisition
time. The operator sends its permitted NMT/SDO recovery traffic through the
existing shared sender.
No hold-to-drive button is required in the default profile (`button_deadman: -1`).
An optional configured deadman still stops immediately when released. In MANUAL,
stick input can command motion without pressing a button, subject to supervisor
readiness and speed limits; keep sticks neutral during startup and mode changes.

Joystick timeout/disconnection clears the stored velocity, deadman/turbo flags,
and acceleration timing. The callback also detects a receipt gap before accepting
a reconnect message, so timer ordering cannot revive an old command. A neutral
joystick remains at zero after reconnection. Held nonzero controls cannot rearm:
fresh neutral input is required before subsequent stick movement is accepted.
A configured deadman additionally requires release and a new press. A backward
clock jump, mode/authority change or safety recovery requires this same rearm.

The temporary USB profile is for Logitech F310/F710, using the hardware XInput
setting and `joy/game_controller_node`. It uses SDL's standardized ROS layout,
not the manufacturer-dependent ordering of `joy_node`. All four face buttons
(indices 0..3) are unused. Verify the actual input with `ros2 topic echo
/moboterra/joy` on a secured platform before operating.
Its `frame_id` must exactly match the supervisor's `output_frame_id` (both default
to `base_link`); mismatched/empty velocity frames are rejected by the supervisor.

With USB selected, bringup starts a separate `joystick` node using the selected
`joy_node` or `game_controller_node` driver to publish `joy`.
`mobotic_manual_control` implements the teleop role from the original diagram;
it is not a replacement name for the joystick driver. The supervisor subscribes
to operator command/state, not directly to `joy`.

## Temporary Logitech controls

For the measured PAIL CAN receiver in virtual mode, the left paddle (A2) steers,
including pre-steering at standstill. The right cross stick uses A3 for
forward/reverse and A4 for crab left/right, with zero yaw when the paddle is
centred. These CAN channel numbers are independent of the USB indices below.
Stationary angle requests pass through the supervisor and are disabled by
default outside the virtual CAN launch.

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

`operator/state.deadman_pressed` reports an actual configured button, not simulated
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

Back's rising edge publishes `VehicleModeRequest` on `operator/mode_request`,
independently of boost. It reads fresh `vehicle/mode`: MANUAL requests
AUTO_VELOCITY, while AUTO_VELOCITY or AUTO_DIRECT requests MANUAL. It never requests
AUTO_DIRECT. No local toggle state is assumed after terminal requests.
During a reported transition, or without fresh supervisor mode feedback, presses
are ignored; release and press again after readiness. Holding does not repeat.
Startup, timeout, invalid layout or reconnect primes the input, so an already-held
button cannot change mode. Fresh, nonzero, ordered Joy timestamps are required
for mode requests. Missing optional mode inputs do not break manual driving.

`button_mode_switch: -1` disables joystick mode requests. `mode_switch_type:
position` supports a two-position RP1 input. The REMdevice provider requires an
explicitly verified physical mapping for motion; an unverified empty mapping
runs in capture-only mode. Only one provider starts. USB reports
`estop_state_known=false`; the physical receiver button must never be inferred
from another safety signal or guessed bit. Receiver mapping remains pending.

The supervisor stops the platform through the outgoing command path, verifies
0.25 s measured standstill, then selects exactly one mode and waits for a new
command. Requests use the same logic as terminal `vehicle/set_mode`; a request
is not confirmation that the transition completed. Observe `vehicle/mode`.
AUTO_DIRECT (2) is accepted only by `vehicle/set_mode`, not the mode-request topic:

```bash
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode "{requested_mode: 2}"
```

This separates supported control APIs; it is not ROS client authentication.
The physical USB mapping and stop behavior still require hardware validation.

The CAN provider decodes the documented unsigned-magnitude/direction format and
supports mapped B2 rising-edge requests to the supervisor's
`wheel_modules/clear_errors`. If a receiver emergency is latched, B2 first
verifies its error register is zero, then requires fresh neutral PDOs before
requesting the supervisor reset. Active receiver faults remain inhibited;
the stack stays running. See [receiver B2 recovery](../../docs/remdevice_commissioning.md#b2-acknowledgement-of-a-recovered-receiver-emergency).
RP2/STOP retain their independent safety functions.
The provider never rewrites PDO mappings, node ID or bitrate. Its explicit
commissioning service can send targeted NMT start; after mapping verification,
targeted NMT startup is automatic. See the guide for hardware acceptance.

ROS-free regression tests:

```powershell
$env:PYTHONPATH="src/mobotic_config;src/mobotic_manual_control"
python -m unittest discover -s src/mobotic_manual_control/test -v
```
