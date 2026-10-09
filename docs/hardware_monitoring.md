# Hardware monitoring and commissioning

ROS observes the hardware safety chain. ELMO/FlexiSoft perform physical STO;
none of the changes here implement or replace a safety-rated stop function.
A driver/OS/CAN failure cannot be handled by a watchdog inside that failed process.
Verify the independent hardware watchdog/STO and brake behavior on the machine.

## Reference configuration

The local `terra_spiro` files were read as reference data; only MoboTerra was edited.

- `mobotic_launch/launch/terraspiro.launch.py`: Vanguard controller/internal
  participant source addresses and status/SOC/electrical PGNs. MoboTerra keeps
  one public `battery/state`, plus `battery/system_state` for readiness.
- `startup/README.md` and `vanguard_charge_monitor.launch.py`: separate 500 kbit/s
  CAN buses, `can0` for drives and `can1` for listen-only battery telemetry.
  MoboTerra bringup now launches a receiver-only battery transport under
  `battery_can`, remapping the monitor input to `battery_can/can_rx`. Neither
  this change nor the read-only monitor adds operating-consent/HV commands.
- `sick_flexisoft_eth_comm/src/sick_flexisoft_eth_comm.cpp`: 15-byte TCP telegram,
  port 9100, byte 14; OSSD masks 0x03/0x0C, estop 0x10, override 0x20, warning
  masks 0x80/0x40. The complete bytes are now published as `flexisoft/telegram`.
- `mobotic_launch/launch/mobotic-etp.launch.py`: scanner settings, front scanner
  10.60.20.52, host 10.60.20.186. Rear 10.60.20.53 was supplied by the user.
- `debug/can_rosbag_recorder.py`: ELMO read-only SDO objects 0x2086:0 (STO status)
  and 0x60FD:0 (digital inputs). They are polled even when drives are disabled.

The scanner API/remappings follow the [SICK ROS 2 driver documentation](https://github.com/SICKAG/sick_safetyscanners2).
Both nodes publish separate scan/raw/output-path topics. They do not feed a new
ROS safety algorithm: protective fields still reach the supervisor through FlexiSoft.
Autonomy can subscribe to the laser scans; no autonomy implementation was added.
Scanner transforms remain a separate URDF/TF task.

The reference battery is a single **system** containing a controller and four
configured internal participants. Every configured participant must send all
three telegrams. If the actual battery has fewer participants, reduce all five
`batteries.*` arrays together; do not leave absent modules configured or readiness
will correctly remain false.
SOC reference units are percent; `sensor_msgs/BatteryState.percentage` and
`minimum_percentage` are fractions. Invalid/reserved values produce NaN and block
readiness, not a clamped 100% SOC. The supervisor threshold remains 0.2 (20%).

Battery status, SOC and electrical freshness use nonzero `battery_can/can_rx.header.stamp`
as acquisition time. Frames already older than `telemetry_timeout` (0.5 s) are
rejected; delayed accepted samples expire at acquisition time plus timeout.
Per-participant/per-telegram ordering rejects duplicate timestamps and older
samples, so an older healthy payload cannot replace a newer fault. Timestamps
up to 100 ms ahead are clamped to receipt time; larger future skew is rejected.
Original timestamp ordering is retained even for clamped samples, preventing
replays from renewing freshness. After a backward clock jump, earlier samples
cannot replace the stored history; fresh telemetry can resume after time catches
up, or after restarting the monitor into the new clock epoch.

Zero-stamped frames use receipt time for compatibility. A bridge repeatedly
replaying unstamped payloads cannot be detected this way; preserve acquisition
timestamps on the actual CAN transport. Battery output headers remain report
times; the monitor's readiness/stale flags account for acquisition age.

Verify the installed `ros2_socketcan` receiver launch supports `use_bus_time`;
MoboTerra explicitly sets it to `true` on both transports. An external battery
receiver must preserve acquisition timestamps and publish on
`/moboterra/battery_can/can_rx`. In integrated bringup,
`start_battery_socketcan:=false` selects that external receiver without stopping
the monitor. `start_socketcan` controls only the drive bridge. The standalone
battery launch instead uses its own `can_interface` and `start_socketcan` arguments.

## Four-wheel drive profile

The hardware/virtual driver YAML and standalone driver defaults share the old
four-wheel heartbeat (0.01 s) and traction ceiling (116053 motor increments/s).
All four traction per-drive limits use the same positive/negative ceiling.
With 16:1 gearing, 4096 counts/revolution and wheel radius 0.325 m,
nominal 1 m/s^2 corresponds to `round(16*4096/(2*pi*0.325)) = 32093`
increments/s^2. Both `traction_profile_accel` and `traction_profile_decel` now
use this value instead of the smaller-wheel default of 92714 (~2.89 m/s^2).
The supervisor's 0.5 m/s^2 command ramp and normal speed limits are unchanged.
Confirm actual acceleration, stopping distance and CAN load on a secured
platform; matching source settings is not hardware commissioning.

## STO visibility limitations

The reference FlexiSoft byte has no independent STO or safety-enable bit.
`SafetyIOState.sto_state_known` / `safety_enable_state_known` therefore remain false
with the supplied configuration. `sto_active` on communication loss or estop is
a conservative ROS interlock, not proof that hardware STO has physically engaged.
`flexisoft_status_raw` records the last byte; use `SafetyState.communication_ok`
to determine whether it is fresh.

ELMO diagnostics expose `sto_0x2086_raw`, `digital_inputs_0x60FD_raw` and their
`*_fresh` flags per traction drive. Unsupported SDOs, aborts and aged responses
are reported as unknown. No raw value is decoded into active/inactive STO until
the hardware bit mapping is supplied and verified. No writes to these objects
are performed, and no new enable prerequisite on an unmapped register is added.

## Supervisor loss behavior

Driver management and motion watchdogs are separate:

- `watchdog_timeout: 0.1`: missing accepted joint setpoints request zero motion.
- `supervisor_timeout: 0.3`: missing accepted `wheel_modules/command` requests
  stop + disable and cancels initialization/enable retries, independently of
  continued joint commands. A steady-clock timer checks the lease every 20 ms.
- Commands must be timestamped and fresh. Replayed/non-increasing management
  timestamps do not refresh the lease. Manual enable services cannot bypass it.
- A fresh supervisor `enable=true` can restore enable normally; traction targets
  and steering profile speed are zeroed before enabling, including retries.
  A fresh motion command is needed to move again.
  A backward ROS clock jump fails closed until time catches up or the driver is
  restarted with a new clock epoch.

These are requested CAN actions, not confirmation that brakes/STO have engaged.
The timeout plus check interval does not include OS scheduling or CAN transport
latency. Commission on a secured platform with a working hardware stop path.

## CAN feedback loss

`WheelModuleStatus.feedback_fresh` is true only when **both** drives in the module
have fresh status, velocity, and error-register responses; steering also requires
position, traction also requires the error-code response. The array header is the
report publication time, not proof that the underlying CAN values are fresh.
`status_message` identifies the first missing/stale field. Cached `enabled` is not
reported true when status feedback is stale. Cached faults are retained as last-known
values; zero error is not proof of health unless `feedback_fresh` is true.

Defaults: motion/status timeout 0.3 s; fault timeout 2.5 s with 1 s error polling.
Status polling continues in every controller state at up to 0.1 s intervals.
These timeouts must exceed the configured polling/PDO sample gaps; increasing
`telemetry_pdo_sync_divider` may require increasing the motion timeout.
Heartbeats, current-only telemetry, STO SDOs, write acknowledgements, wrong
objects/subindices and malformed frames do not refresh required feedback fields.

The supervisor inhibits motion and standstill-based mode switching on stale
module feedback. A separate driver-side steady-clock check (20 ms) also rejects
joint commands and requests stop on **all** units when any required feedback is
missing/stale, a unit is disabled, or a known fault is present. Stop requests are
repeated at most every 0.1 s while blocked, and traction PDO targets are kept zero.
This does not change the requested enable rule (safety/battery readiness); SYNC,
status and fault polling continue so feedback can recover without a startup deadlock.

Recovery requires fresh required fields and enabled, fault-free drives. Old motion
targets stay cleared and joint commands predating recovery are rejected. The
supervisor also clears every source-command buffer on readiness loss and recovery,
rejects commands received while inhibited, and requires a source timestamp at or
after recovery before forwarding motion. Hardware
STO remains independent. Stop transmission to a disconnected drive cannot guarantee
physical stopping; verify hardware protections and the healthy-unit stop behavior.

Supervisor gates recheck receipt age and original header age on every use of
cached commands and readiness. A recently delivered old sample expires at its
original timestamp plus timeout, not receipt plus timeout. Safety/battery expiry
disables drives; wheel/direct-joint feedback expiry inhibits motion while retaining
the safety/battery-only enable policy. Negative receipt age fails closed. Header
checks retain the existing 100 ms future-skew allowance; standstill remains strict.

`joint_states` is published only for a complete fresh motion sample set and its
timestamp reflects the oldest required motion sample, not the aggregation time.
Optional effort is omitted if any current sample is unavailable/stale. CAN bridge
acquisition timestamps, when supplied, contribute to sample age; zero timestamps
fall back to local reception time. Do not use a CAN transport that repeatedly
replays unstamped old payloads, which cannot be distinguished from real samples.

## Diagnostics and launch

Driver, platform monitor and native scanner diagnostics share the namespaced
`diagnostics` topic (default `/moboterra/diagnostics`). Platform reception
timeouts use a steady clock and also check message header age.

Coverage: supervisor lease, joint-command watchdog, per-drive CAN freshness and
faults, CAN error frames, raw ELMO safety registers, battery telemetry/readiness/SOC,
FlexiSoft readiness/unmapped signals, mode/transition status, joystick and both scans.
Warnings for unmapped signals are expected; they must not be interpreted as
verified hardware safety. Diagnostics only observe and never issue commands.

Hardware bringup passes `filters=0:0,#1FFFFFFF` to `ros2_socketcan`: normal data
remains unfiltered and all error classes are subscribed. The bridge's default
`0:0` data filter alone leaves its error mask zero. The supported syntax is in
the [bridge parser](https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan/src/socket_can_receiver.cpp);
the [Linux SocketCAN contract](https://docs.kernel.org/networking/can.html#raw-socket-option-can-raw-err-filter)
describes the separate error subscription. Driver error frames increment the
`/can_errors` diagnostic counter and are never decoded as motion feedback.
This does not configure bitrate, bus-off recovery, drive reset or enable.
An external bridge used with `start_socketcan:=false` must opt in similarly.
No observed error frames must not be treated as proof of bus health.

The platform node subscribes to `battery/system_state` (0.6 s), `safety/state`,
`safety/io_state` and `vehicle/mode_state` (0.3 s), optionally `manual/state`
(0.3 s) and both `scanner/<location>/scan` streams (0.5 s). It publishes at 2 Hz.
It does not directly subscribe to `joint_states`, `wheel_modules/status`,
`battery/state`, `agv_vel` or CAN; driver diagnostics cover CAN/wheel freshness.
Scan loss raises diagnostics only; FlexiSoft protective fields remain the
supervisor's scanner-related motion interlock.

Hardware bringup starts both scanners automatically. Confirm the PC NIC actually
uses the configured address; override `scanner_host_ip` if necessary:

```bash
ros2 launch mobotic_bringup moboterra.launch.py scanner_host_ip:=10.60.20.186
ros2 topic echo /moboterra/diagnostics
ros2 topic echo /moboterra/safety/io_state
ros2 topic echo /moboterra/flexisoft/telegram
ros2 topic hz /moboterra/scanner/front_left/scan
ros2 topic hz /moboterra/scanner/rear_right/scan
```

`start_scanners:=false` disables the scanner nodes and their scan-health checks.
Virtual bringup never starts physical scanners; safety/battery remain explicitly mock.
Virtual drives implement the production MiControl/ELMO SDO, enable, NMT and
synchronous PDO paths with the same PDO configuration as hardware mode. Protocol
tests cover all eight units' required feedback. There is no physical motor/brake/
STO model, and ROS full-stack integration is not yet validated. Mock readiness
does not bypass required CAN feedback gates. Unknown-register warnings are expected.

## Verification

On the Windows development host, run the ROS-free suites:

```powershell
python -m unittest discover -s src/mobotic_supervisor/test -v
python -m unittest discover -s src/mobotic_manual_control/test -v
python -m unittest discover -s src/mobotic_vanguard_battery/test -v
python -m unittest discover -s src/mobotic_safety/test -v
python -m unittest discover -s src/mobotic_bringup/test -v
python -m unittest discover -s src/mobotic_description/test -v
```

On the ROS 2 build host, rebuild interfaces and dependents and run the C++ lease
and feedback-decoder/freshness tests (the message definition changed):

```bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select mobotic_driver
colcon test-result --verbose
```

Install the `sick_safetyscanners2` ROS package and its dependencies on that host.
Test supervisor loss with wheels safely lifted/secured, not during normal driving:
terminate only the supervisor process, ensure no automatic respawn during this test,
and confirm CAN disable requests and physical stopping/braking while a joint
publisher continues. Bringup normally enables supervisor respawn, so a short
restart may complete before the timeout. Also test low SOC, battery telegram loss,
FlexiSoft disconnect, occupied fields and missing scan streams separately.
Test joystick loss while commanding motion: confirm zero output and inactive
manual state after the timeout, then reconnect with neutral axes and no buttons
pressed. Output must remain zero. With nonzero input, verify acceleration restarts
from zero rather than the pre-disconnection target.

For CAN feedback tests, selectively suppress one unit's motion/status/fault replies
on a secured test setup while keeping heartbeats, other units and supervisor commands
alive. Confirm `feedback_fresh=false`, blocked mode/motion, zero targets for all units,
and no fresh `joint_states` carrying the expired sample. Restore only unrelated
traffic first: it must not restore motion. Then restore all required replies and
confirm motion resumes only from commands stamped after recovery. Exercise both
SDO and PDO feedback paths and test the fault-read timeout separately.
Also stop the autonomy command publisher before restoring feedback: recovery must
remain stopped until a new post-recovery command arrives. Repeat for autonomous
velocity and direct joint modes; replaying a pre-recovery command must not move.
Test delayed command delivery too: with the default 250 ms timeout, inject a
command stamped 240 ms before receipt. It must stop after the remaining 10 ms
(on the next control cycle), not stay valid for another 250 ms. Repeat for manual
velocity and both autonomy paths. Delayed safety/battery readiness must likewise
expire based on original age, even if reception was recent.
On a secured setup, send a fresh autonomous velocity command with `frame_id=map`
or an empty frame: it must be rejected, clear any prior autonomous command, and
produce zero on the next control cycle. Then send a properly body-frame command
with `frame_id=base_link` (or the configured `output_frame_id`) and verify normal
supervised behavior. Repeat with the manual velocity input. Drive enable remains
governed only by safety/battery readiness, and direct joint routing is unchanged.
For battery CAN, inject status/SOC/electrical frames acquired 490 ms before receipt
with the default 500 ms timeout. The pack must become stale after the remaining
10 ms (at the next publication), not receive another 500 ms. Frames already older
than 500 ms, duplicates and older reordered samples must not restore readiness
or overwrite a newer fault. Restore genuinely newer telegrams for every configured
participant to recover. Verify zero-stamp compatibility separately; it cannot
provide delayed-acquisition/replay detection.

Full ROS integration, C++ compilation and hardware fault-injection are not
validated by the Windows-side unit tests.

On the Linux/ROS host, verify the installed bridge supports its `filters` launch
argument and check the applied parameter:

```bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml --show-args
ros2 param get /moboterra/socket_can_receiver_node filters
```

Expect `0:0,#1FFFFFFF` after hardware bringup. On an isolated test interface or a
secured commissioning setup, verify error frames appear on `can_rx` with
`is_error=true`, increment the driver's `error_frame_count`, and raise its recent
error warning. They must not refresh wheel feedback or battery readiness. Confirm
normal standard drive replies still pass on `can_rx`; extended J1939 battery
frames must appear separately on `battery_can/can_rx` and update the monitor.
Do not create bus faults on an operating vehicle to test diagnostics. Error
generation/reporting depends on the kernel CAN driver and hardware; a static
launch test does not validate that end-to-end path.

## Current validation and outstanding work

The ROS-free Python suites currently contain 223 tests (supervisor 72, manual control 29,
battery 21, safety 20, bringup 32, description 21, odometry 28). Battery launch checks cover
separate interfaces/namespaces, receive-only transport, acquisition timestamps,
hardware/virtual conditions and external-receiver selection. Profile checks
cross-check acceleration units, all traction limits and standalone/C++ defaults.
These launch/configuration checks do not run ROS or validate CAN hardware.
Mode regressions cover all nine mode pairs (including reselection), moving/stale
feedback, simultaneous sources, topic/service parity and conflicting requests.
Joystick tests cover Back-button edges, held/reconnected inputs, authoritative
mode feedback, terminal-only direct selection, unused face buttons, independent
crab translation, steering signs, timestamp validity and mapping overlap.
No-hold driving tests verify both shoulder boost inputs, non-stacking boost,
unused left-stick vertical input, inactive invalid layouts and optional-deadman
behavior. Supervisor tests verify that no-hold manual motion still requires a
fresh connected/active source and all hardware readiness checks.
Bringup uses `joy/game_controller_node` standardized SDL ordering for the temporary
F310/F710 USB profile; the raw `joy_node` index ordering is not interchangeable.
The driver-side motion-source gate passed
four standalone C++ regression groups with address/undefined-behavior sanitizers;
these cover exclusive routing, transition epochs, replay, delayed mode authority,
paused/backward clocks and recovery. The actual ROS driver/executor still requires
a target-host build and secured integration tests.
In addition, the kinematics
production-cache C++ regression executable passed 10 test groups under the local
Ubuntu compiler with address and undefined-behavior checks enabled (leak tracing
disabled for WSL compatibility). It covers delayed/stale
feedback, paused ROS time, replay, malformed complete messages, atomic updates and
recovery. The full ROS kinematics node has not been built/run here.
The production CAN decoder/ordering helpers also passed 15 C++ regression groups
with the same sanitizer checks, compiled using a test-only CAN message-shaped
adapter. They cover per-field PDO/SDO ordering, duplicate acquisition stamps with
conversion jitter, fault/disable replay, current/mode data, independent drives,
reset retention and raw read/abort ordering. ROS builds use actual `can_msgs`;
the adapter does not validate ROS serialization, subscriptions or the driver node.
The steering-target backend passed six additional C++ regression groups with the
same compiler/sanitizers. They cover signed-32-bit and half-tick boundaries,
multiple resolutions, non-finite/huge values, intermediate overflow, unchanged
multi-turn commands and existing explicit tick limits. Supervisor regressions
cover all four steering joints, permuted joint order, invalid replacement stops,
hold-feedback rejection and matching supplied encoder-resolution configuration.
The safety bridge tests also pass on Linux and Windows with real loopback TCP
sockets and ROS adapters. They cover occupied-port failure/recovery, injected
bind/listen/accept errors, unexpected worker/publisher failure, fragmentation,
disconnect/reconnect, active-receive shutdown and main cleanup/error propagation.
Launch-contract tests check the 1 s bridge respawn delay in both launch files.
These checks do not run the actual ROS executor or launch process manager.
Odometry tests exercise the production Python SE(2) integrator, covariance
Jacobians, straight/crab/curved/pure-rotation motion, time ordering, paused and
backward clocks, delayed acquisition, gap rebasing and reset watermarks. Callback
adapters check pose/TF/stamp consistency, frame rejection, TF-disable selection,
standstill reset gating and stale/degraded diagnostics. Launch/package checks
cover TF namespacing, default bringup wiring and explicit no-respawn policy.
They do not validate ROS transport, localization compatibility or wheel slip.
The virtual protocol core passed 12 C++ regression groups with the same sanitizer
checks and test-only CAN adapter. They exercise production request/decoder round
trips, all-eight-drive readiness, steering movement, signed ELMO feedback widths,
SYNC/PDO layouts/dividers, faults, pending-target cancellation, mapping/NMT and
malformed-frame isolation. The ROS wrapper has not been built/run here.
C++ driver lease and decoder/freshness tests are registered but
have not been compiled/run on this Windows host. Rebuild interfaces and all
dependents before checking the ROS graph or hardware behavior.

Kinematics now requires complete joint feedback fresh by both original timestamp
and steady receipt age (`joint_feedback_timeout: 0.3`). Invalid newer feedback
blocks commands; duplicate/older samples cannot renew the cache. On a secured
setup, interrupt only the delivery of `joint_states` to kinematics while other
driver telemetry continues. Kinematics must stop publishing joint setpoints and
the driver's configured motion watchdog must stop its previous targets. Restore
genuinely newer complete feedback, then verify normal command processing. Repeat
with a delayed sample near the timeout boundary and a malformed last module;
neither may extend feedback lifetime or partially update steering positions.

On an isolated test transport (or a secured setup with physical motion prevented),
verify timestamped drive CAN ordering: after a newer fault/error-code response,
replay older and identical-stamp healthy replies. The fault must remain visible,
and the replies must not renew field freshness or trigger initialization actions.
After a newer disabled-status SDO, replay an older enabled-status PDO. Disabled
state must remain authoritative, though genuinely newer velocity from that PDO
may be accepted independently. Repeated identical-acquisition-time motion frames
must still expire at the configured timeout, even if ROS receipt/conversion times
change. Consuming a joint-state batch must not remove replay protection. For raw
`0x2086`/`0x60FD` telemetry, a newer abort must remain unavailable despite older
read replies; an older abort must not erase a newer read. Delayed raw telemetry
must retain acquisition age. Restore genuinely newer responses to recover.
Zero-stamp transports retain a local-time fallback but cannot prove replay order;
use the configured acquisition-stamping bridge for hardware commissioning.

On an isolated ROS/CAN test setup, submit an otherwise valid complete command
with an unrepresentable steering angle on the last joint. No part of that batch
may produce a CAN target write or renew the driver's motion watchdog. Repeat on
both driver motion inputs. In selected `AUTO_DIRECT`, an invalid replacement on
`autonomy/joint_setpoints` must clear the prior candidate and request zero
velocity/current on the next supervisor cycle. Keep
`direct_steering_encoder_resolutions` aligned with the driver's steering
`drives.resolutions`. Numeric CAN bounds are not mechanical steering limits;
verify physical travel and calibration before operating the vehicle.

On a secured ROS integration setup, occupy the configured FlexiSoft TCP port
before launching bringup. The bridge must log a fatal listener error, exit with
nonzero status and retry with a 1 s respawn delay, not leave an inert ROS process.
Release the port and verify that a new bridge instance accepts the hardware client.
Without new complete telegrams, `safety/state` must remain inhibited after the
configured 0.25 s status timeout; a process restart is not readiness evidence.
Also disconnect/reconnect the TCP client: the listener should stay running and
discard partial telegrams from the old connection. Stop launch with a client
connected and confirm clean shutdown without an unintended restart. Unexpected
worker termination is checked every 0.1 s using steady time even if ROS time pauses.

## Wheel odometry commissioning

Bringup now starts `mobotic_odometry` from measured `agv_vel`, producing
`/moboterra/odometry` and optional `odom -> base_link` on `/moboterra/tf`.
No command velocity is integrated. The local odom plane starts at base_link
height; the URDF's chassis/floor offsets are not applied a second time.

On a secured ROS setup, verify straight/reverse and sideways motion signs,
pure rotation and mixed curves. Compare measured distance/yaw with external
measurements and tune wheel radius, mounting calibration and covariance.
Check that odometry pose and TF match with identical acquisition timestamps;
body twist must remain in base_link while pose is expressed in odom.
Run only one broadcaster for this TF edge; use `publish_odom_tf:=false` when
an external EKF/localization system owns it. RViz needs namespaced TF remappings.

Interrupt measured `agv_vel`: odometry and TF publication must stop, and both
node/platform diagnostics must age stale. Restore feedback: the first new sample
must hold the previous pose, increase uncertainty and establish a new integration
baseline, without guessing missing motion. The node reports persistent WARN for
this uncertainty. Duplicate/older samples must not publish or refresh the estimate.
Test paused ROS time, delayed delivery, a backward clock jump and source loss
without disabling the independent hardware stop chain.

The node is intentionally not automatically respawned: a restart or approved
`odometry/reset` changes the origin, which downstream consumers must handle.
Reset requires fresh measured standstill and does not replay prior samples.
Defaults are uncalibrated wheel-only uncertainty, not safety-rated accuracy.
See [odometry interfaces and limitations](../src/mobotic_odometry/README.md).

Still pending: secured hardware validation of configuration/scaling and loss
behavior, virtual-stack ROS runtime validation, an explicit fault-reset/retry policy,
joystick fault-reset controls, the final REMdevice CANopen receiver/TPDO mapping,
and ROS/hardware odometry/TF validation. The temporary F310/F710 USB profile uses
Back to toggle MANUAL/AUTO_VELOCITY and no face buttons. Left stick horizontal
steers; right stick translates/crabs. No held shoulder is required to drive;
L1 or R1 boosts, with no stacking when both are pressed. Left-stick vertical is
unused. Set the gamepad to XInput and verify axes/buttons on a secured platform.
Joystick mode requests use the same verified stop/switch transitions as the terminal.
AUTO_DIRECT is selectable only through `vehicle/set_mode`,
without an extra opt-in, and the driver rejects inactive-source setpoints using
fresh mode authority. Test all mode pairs and mode-state loss on secured hardware.
The updated
[robot description](../src/mobotic_description/README.md) has a standalone joint
TF launch with user-confirmed CAD floor/axis placement, but ROS/RViz and hardware
validation are pending; it is not integrated into default bringup. The description
now includes nominal CAD/manufacturer-aligned `front_left_scan` and
`rear_right_scan` fixed frames. Their measurement planes are 0.249261344 m above
the CAD floor, facing +45/-135 degrees respectively. The CAD coverage disks are
16.9 mm below the actual scan plane and must not be used as laser-height datums.
Physical extrinsic calibration and corner/IP verification remain required;
see the description's scanner-frame evidence and commissioning notes.
The four-module update uses reference CAN bindings (front-left 0x01/0x02,
front-right 0x03/0x04, rear-left 0x07/0x08, rear-right 0x05/0x06). Verify these
addresses, all eight axes' motion signs/scaling, and the four-wheel reference
mounting offsets before operating. User-confirmed CAD spacing is 1.69 m / 0.99 m,
wheel radius is restored to 0.325 m, and CAD Z=0 is the floor. That does not
validate CAN mapping or steering calibration. Old two-diagonal 0.08 rad offsets
are no longer used. Modeled steering-datum-to-wheel-axle drop is approximately
0.425 m, derived from the CAD datum and floor + radius, not from the old zero-offset URDF.
The unknown STO mapping cannot be resolved without verified hardware documentation.
See the [source-derived project graph](moboterra_rqt_graph.md).
