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

Manual input uses neutral rearm and fresh commands. Hold-to-drive button support
has been removed from both USB and CAN input.

Converts `sensor_msgs/msg/Joy` into a supervised `operator/cmd_vel` command source for the vehicle
supervisor. It also publishes `mobotic_interfaces/msg/OperatorState` on `operator/state`.

The node never calls drive services. A stale joystick or invalid joystick layout immediately produces
a zero command and an inactive state. The supervisor remains the only component allowed to select this command
source and authorize motion.

For CAN, `operator/state.connected` requires fresh physical LINK/EMGY/STOP
evidence and a receiver without an active EMCY/input fault. An EMCY withdraws
drive readiness and reports its code with joystick/receiver restart guidance;
the supervisor disables drives and waits for B2 after the fault is corrected.
An incomplete PDO pair sets `command_active: false`, discards motion, and
requires fresh neutral rearm without fabricating physical STOP state.
The physical status retains its own digital acquisition time and expires after
250 ms; EMCYs and publishing timers cannot refresh it. STOP/radio loss still
inhibit immediately. Once the receiver is healthy, supervised zero-target
controller initialization can proceed under B2 authorization while the motion
source rearms, without accepting an old command.
The measured RP2 selector is published separately on `operator/scanner_selector`
with its digital acquisition timestamp and no motion axes. Publishing that
physical switch state never feeds the teleop callback or rearms motion.
Hardware bringup uses two receive-only SocketCAN channels: `operator_rx/can_rx`
contains the two control PDOs, while `operator_events/can_rx` contains receiver
EMCY, SDO and heartbeat messages. Both retain CAN errors and acquisition stamps.
Separate bounded DDS histories prevent diagnostic bursts from evicting control
PDOs; queued samples still have to meet the existing 250 ms age and 75 ms pair
skew limits. Receiver faults and STOP retain their existing inhibition behavior.
The operator sends permitted NMT/SDO recovery traffic through the existing shared
sender. The extra reader does not transmit CAN frames. Virtual input stays on
the isolated `operator_can` topics.
In MANUAL,
stick input can command motion without pressing a button, subject to supervisor
readiness and speed limits; keep sticks neutral during startup and mode changes.

Joystick timeout/disconnection clears the stored velocity, turbo state,
and acceleration timing. The callback also detects a receipt gap before accepting
a reconnect message, so timer ordering cannot revive an old command. A neutral
joystick remains at zero after reconnection. Held nonzero controls cannot rearm:
fresh neutral input is required before subsequent stick movement is accepted.
A backward clock jump, mode/authority change or safety recovery requires this same rearm.

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

The measured PAIL CAN mapping uses A2 for left-stick steering and A3/A4 for
right-stick forward/reverse and sideways translation. These channel numbers are
independent of the USB indices below. Left-stick input alone does not steer or
rotate the vehicle. Manual steering families are selected by the first qualifying
steering input; crab wins if both qualify in the same acquisition sample.

| Physical control | ROS index | Role |
|:--|:--|:--|
| Left stick horizontal | axis 0 | Steering while moving; yaw reverses with travel direction |
| Right stick vertical | axis 3 | Forward/backward translation |
| Right stick horizontal | axis 2 | Left/right crab translation, no yaw |
| L1 / left shoulder | button 9 | Boost, temporary B1 role |
| R1 / right shoulder | button 10 | Same boost; either shoulder works |
| Back | button 4 | Toggle MANUAL (0) / AUTONOMOUS (1), temporary RP1 role |
| Left stick vertical, face buttons, Start, stick clicks, D-pad and triggers | unused | No assigned function |

Ackermann uses left-stick horizontal plus right-stick forward/reverse; right-stick
sideways input is ignored after Ackermann wins. Curvature derives from configured
wheel coordinates so full left-stick throw reaches 45 degrees on the innermost
wheel; outer wheels use the matching smaller angle. Front and rear steering retain
the existing coordinated four-wheel model, with opposite angles for a common turn
centre. Reversing changes yaw sign with travel direction.

Crab uses only the right-stick vector and ignores the left stick. Sideways-only
input drives sideways at a 90-degree heading; diagonal input gives the corresponding
intermediate heading. Every wheel stays within its configured crab envelope. Straight travel
without steering does not reserve a family. After selection, only neutral owning
controls plus fresh measured body AND individual traction standstill release it.
Returning the ignored stick to centre is not required. A connection/authority
reset discards the selection and old motion commands. `operator/state.status_message`
reports `steering=ackermann`, `crab`, `straight` or `idle`.

Manual-only and supervised manual speed caps scale forward, lateral and yaw
velocity together. Normal/boost speed or a reduced-speed condition therefore
does not change the requested Ackermann turning radius or crab direction.
For example, a 1 m/s request with 0.22827 rad/s yaw becomes 0.3 m/s with
0.06848 rad/s yaw at the 0.3 m/s cap; the front-left steering target remains
about 12.27 degrees instead of jumping to 45 degrees. Steering travel and the
supervisor's acceleration ramps still take time to follow changing input.

Final manual wheel commands are checked in both authorities, for native and romb
backends. Targets use the nearest full-turn home and account for pi-mounted right
units and their traction polarity. Traction follows the requested travel command while steering moves; there is
no wait-for-heading traction gate. Like the example project, steering headings
use the previous requested positions for continuity. An equivalent heading that
would need more than 90 degrees of steering instead reverses traction. Reversing
full sideways Crab therefore keeps the four steering targets and reverses travel,
rather than sweeping the steering from +90 to -90 degrees.

Crab uses one shared rolling-axis choice for all four modules, including
pi-mounted right motors. Starting directly sideways and approaching sideways
gradually choose the same controller-angle branch. When an up-to-sideways-to-down
stick sweep reaches +/-90 degrees, steering stays at that endpoint and traction
is zero beyond the boundary. A new inward heading releases the boundary,
including straight forward or reverse travel at 0 degrees and inward reverse
diagonals. This does not require a neutral step or completion of the old turn.
Outward requests remain blocked. The optional platform setting
`motion_limits.max_crab_angle` (radians, from pi/2 through pi) widens this envelope.
Bounded profiles can use +/-180 degrees: a continuous sweep can pass sideways
and reach a backward-facing wheel heading. Straight input at a +/-90-degree tie
still chooses the inward 0-degree axis.

The active robot sets `motion_limits.steering_angle_limits_enabled: false`.
This removes absolute homing-relative angle caps for both manual steering
families. Steering follows the nearest equivalent heading around the previous
target, reversing traction when that avoids a large steering change, as in
Terra Spiro. A gradual joystick sweep can therefore accumulate continuous turns.
With neutral homing enabled, releasing the controls returns to the nearest
full-turn home with zero traction. The Ackermann/crab mode
lock and the existing joystick curvature mapping remain unchanged. Profiles
with angle limits enabled retain their configured crab and +/-45-degree
Ackermann envelopes.
Without neutral homing enabled, neutral holds measured positions. Interlocks
and stale input always hold position and cannot initiate a return-home move.

Fresh neutral stick input cancels the source velocity immediately, including
any remaining acceleration-ramp state. A request above the reduced-speed cap
must not keep the wheels moving while that request ramps down. Starting motion
still uses the configured acceleration limit, and the steering-family lock
remains until measured standstill. Releasing the selected family's motion stick
also stops with an ignored or steering stick still held. Physical stopping follows the drive's
deceleration settings. With angle limits disabled, entering crab retains each
controller's existing turn/traction branch; feedback either side of 180 degrees
cannot introduce an extra full revolution after an arbitrary-angle restart.

The active profile enables `steering_return_home_on_neutral` and sets
`steering_neutral_home_velocity: 1.2` rad/s. Fresh all-controls-neutral evidence
and a fresh zero manual command authorize this return. First a zero batch stops
traction and the previous steering move; a newer complete feedback sample must
show every traction joint within 0.02 m/s before steering moves home. Nearest
full-turn targets are frozen once per neutral period, with half-turn ties toward
the original zero. Each steering profile stops within its encoder tolerance.
Fresh stick input cancels the home targets and resumes the normal 0.3 rad/s policy.

Both kinematics backends publish complete wheel-target batches as fresh velocity
commands arrive, using the latest fresh, complete joint feedback. Publication is
not throttled to the slower physical SDO feedback cycle: the driver retains its
100 ms command watchdog. Source timestamps, command expiry and authority checks
remain enforced.
Every moving steering motor receives the same configured profile-speed limit,
as in the example. Each motor's profile becomes zero when its target is reached
within one degree of the native reachable target, or a finer configured
one-count tolerance. Drift or overshoot beyond this tolerance resumes correction
toward the retained target, including at an envelope endpoint. Reaching a target
does not disable the steering drive or discard its reference. Targets used for
continuity remain unrounded, so repeated corrections do not accumulate rounding
error. The driver applies a zero profile while within tolerance and skips another
position-move request while traction continues. The installed 2.98-degree native
position step still limits physical accuracy; a one-degree deadband cannot measure
sub-count drift or remove the native target's up-to-1.49-degree rounding error.

Steering bounds retain a fixed full-turn reference for each homing/authority
cycle; moving feedback cannot shift it. Physical-heading and controller-coordinate
targets stay inside +/-90 degrees for Crab and +/-45 degrees for Ackermann,
including rounded pi mounting offsets. Ackermann keeps the geometry-dependent
inner/outer angles; Crab rolling axes stay parallel. Shared command timing and
profile limits do not guarantee identical physical arrival times: controller
ramps, CAN delivery and motor response still apply. Existing drive watchdogs and
STOP/E-stop remain active.

The supervisor still applies planar speed and safety limits. L1 or R1 boosts while
held; pressing both does not stack the boost. Boost changes travel speed, not the
45/90-degree steering ceilings, and cannot bypass platform limits.

The supervisor requires fresh `connected` and `command_active` flags. The existing
`OperatorState.deadman_pressed` and `deadman_state_known` fields remain false for
message compatibility; they have no input mapping or permission logic.

Standardized indices are documented by the
[ROS joystick driver](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md).
The F310's input-mode switch is described in
[Logitech's guide](https://www.logitech.com/assets/47879/f310-gamepad-quick-start-guide.pdf).
Keep the gamepad's stick/D-pad MODE setting in normal stick operation and verify
axes on the target Linux machine. Do not use this YAML with raw `joy_node` ordering.

## Mode selection

Back's rising edge publishes `VehicleModeRequest` on `operator/mode_request`,
independently of boost. It reads fresh `vehicle/mode`: MANUAL requests
AUTONOMOUS, while AUTONOMOUS or KINEMATICOFF requests MANUAL. It never requests
KINEMATICOFF. No local toggle state is assumed after terminal requests.
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
KINEMATICOFF (2) is accepted only by `vehicle/set_mode`, not the mode-request topic:

```bash
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode "{requested_mode: 2}"
```

This separates supported control APIs; it is not ROS client authentication.
The physical USB mapping and stop behavior still require hardware validation.

The CAN provider decodes the documented unsigned-magnitude/direction format and
supports mapped B2 rising-edge requests to the supervisor's
`wheel_modules/enable_drives`, matching Logitech X in the example project.
B2 initializes/enables; repeated presses during initialization or while enabled
leave the request unchanged. It never disables or cancels startup. Enabling requires fresh
neutral controls, healthy receiver input and supervisor approval. B2 never
reads or clears receiver error registers or restarts receiver communication.
Held/reconnected B2 does not retry: release and press again. After a response,
motion requires new neutral input. Blocking receiver faults remain inhibited;
the stack stays running. See [B2 drive enable](../../docs/remdevice_commissioning.md#b2-drive-enabledisable).
RP2/STOP retain their independent safety functions.
For a verified PAIL profile, `operator.receiver_errors_advisory: true` makes
receiver EMCY codes, current error-register values and CAN-error indications
diagnostic only. The supplied default platform enables this PDO-based policy.
Diagnostic reports and their clears do not discard controls, reset the
receiver or require joystick rearm. Readiness follows a fresh, valid digital
and analog PDO pair, including acquisition age, skew, axis validation and
LINK/EMGY/STOP. Missing or invalid controls still stop motion; the supervisor's
250 ms operator timeout and drive recovery policy remain active. Raw diagnostics
retain the receiver code/register, policy flag and CAN-error indication count.
Healthy diagnostics stay in `operator/raw`; they do not replace normal operator
status or generate warning/error logs. Actual PDO/STOP/link failures are still reported.
This setting does not repair receiver faults or verify a CAN frame's producer.

Without that policy, `operator.receiver_can_overrun_advisory: true`
allows only `0x8110` / register `0x11` to remain a visible warning while fresh
validated controls continue. The default is `false`. This does not renew input
timestamps or bypass stale/missing PDOs, invalid axes, STOP, LINK/EMGY, CAN-error
frames or hardware safety. Collision `0x8150` and other faults remain blocking.
The provider never rewrites PDO mappings, node ID or bitrate. Its explicit
commissioning service can send targeted NMT start; after mapping verification,
targeted NMT startup is automatic, rate-limited to once per second while the
state is unknown, boot-up, Stopped or Pre-operational. Valid TPDOs or an
Operational heartbeat establish Operational state. Missing/stale local input
or a CAN error invalidates commands without implying an NMT state change or
causing another Start. Heartbeats never refresh command samples. See the guide
for hardware acceptance.

`receiver_auto_recovery` defaults to `true` for verified CAN mappings. The
provider reads the receiver's actual `0x1001:0` error register at startup and
at most once per second, including while PDO input is healthy. This catches
faults whose EMCY predates subscription, establishes a current-register
baseline and distinguishes device health from a software default of zero.
`operator/raw` includes the last matched register-read value and acquisition
timestamp (both null before the first accepted response), plus the last known
NMT state and its acquisition timestamp. These timestamps are observations;
publishing diagnostics does not renew them. A fresh matching zero response clears a
missed EMCY reset and discards old controls for blocking-fault recovery.
`operator.receiver_auto_communication_reset` defaults to `false`: monitoring is
read-only and never clears receiver history or resets the device. B2 only calls
the supervisor's drive-enable service; it cannot trigger either a receiver read
or a receiver reset. If communication reset is explicitly enabled in the profile,
for a blocking `0x8150` with register `0x11`, it can reset the mapped receiver only, with fresh neutral
radio inputs, a current supervisor report confirming disabled drives and no
pending enable/toggle. It waits 500 ms before targeted NMT start. Resets are
limited to three per process and spaced at least five seconds apart. Motion
requires new neutral inputs after recovery; drive enable remains supervisor
owned. Under strict policy, persistent faults, including EEPROM errors, stay inhibited. No EEPROM,
PDO mapping, node-ID or bitrate writes are performed. Set this parameter to
`false` to retain passive receiver-fault handling.

The supplied REMdevice CANopen manual, C-26, specifies that `0x1029:1`
defaults to `1` (No Change on communication error). The receiver can therefore
remain Operational and transmit PDOs while reporting a communication fault.
PDO transmission is not an error-clear indication. C-24 defines `0x8150` as
Collision of Transmission COB-ID, distinct from `0x8110` CAN Overrun Error;
an active collision remains blocking even with fresh PDOs. A matching current
zero register or a zero-code/zero-register EMCY clears the cached diagnostic;
new neutral controls and B2 are still required for drive recovery.

ROS-free regression tests:

```powershell
$env:PYTHONPATH="src/mobotic_config;src/mobotic_manual_control"
python -m unittest discover -s src/mobotic_manual_control/test -v
```


### PAIL diagnostics during an accepted homing sequence

`operator/homing_state` supplies timestamped physical interlocks independently
of the command decoder's EMCY invalidation. It is used only during an already
accepted B2 reset/homing sequence in MANUAL, in supervised and manual-only
operation. Initial B2 admission and normal driving still use `operator/state`.
The observed PAIL communication diagnostics 0x8110/0x8150 with register 0x11 do
not themselves cancel homing while complete fresh physical PDOs continue to
prove neutral controls, LINK/EMGY healthy and STOP released. They remain visible
in the ordinary operator state and raw diagnostics.

STOP/E-stop, radio/link loss, stale/malformed physical inputs, non-neutral sticks,
CAN error frames, other/unknown receiver faults, drive faults or invalid feedback
still inhibit/cancel the sequence. Existing driver authority leases and homing
velocity/time limits are unchanged. The homing topic never publishes an active
motion command and is not used for autonomous or normal joystick motion.
After homing, ordinary operator readiness is required for motion again.
