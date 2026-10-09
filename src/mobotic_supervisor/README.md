# mobotic_supervisor

The MoboTerra command arbiter and vehicle-mode state machine.

The supervisor accepts manual and external-autonomy command candidates, but publishes motion only while safety,
battery, wheel-module, and selected-source state are all fresh and ready. Any invalid, stale, missing, or inhibiting
input causes an immediate zero command. Mode changes require a controlled stop
and verified standstill before the active command source changes.

Both the aggregate `safety/state` and raw `safety/io_state` must be fresh. Emergency stop, inactive safety enable, STO,
or uncleared protective fields in either view inhibit motion. Warning fields in either view apply reduced speed.

Every use of cached commands/readiness checks both receipt age and original header
age against the relevant timeout. Delayed delivery does not start a new validity
window; changing only receipt time cannot keep an old command active. Missing/zero
timestamps are invalid and negative receipt age fails closed. The existing 100 ms
future-header skew allowance is retained; standstill proof still requires a
non-future timestamp. For example, a velocity command arriving 240 ms old has
only 10 ms remaining under the default 250 ms timeout.

`manual/cmd_vel` and `autonomy/cmd_vel` must have `header.frame_id` exactly equal
to `output_frame_id` (default `base_link`). Empty/mismatched frames are rejected
and clear that source's cached velocity, producing zero on the next control cycle
until a valid command arrives. Rejected frames are logged. No TF conversion is
performed: autonomy must express velocity in the body frame before publishing,
not merely relabel a `map`/`odom`-frame vector. Frame checks do not alter drive
enable or direct joint-command routing.

Every control cycle publishes `WheelModuleCommand` on `wheel_modules/command`.
`enable` is true whenever safety and battery are fresh and ready, including when
the selected motion source is inactive or the drives are still disabled. Otherwise
it is false. Battery SOC below 20% inhibits enable; exactly 20% is allowed.
Wheel feedback and command-source readiness additionally gate motion. Normal
enable commands set `clear_errors=false`.

Battery gating uses `battery/system_state`, not public `battery/state`.
Every expected wheel module must be enabled, fault-free and `feedback_fresh=true`;
a newly published array of stale cached CAN values is not ready feedback.
Loss of safety, battery, or wheel readiness clears all buffered motion commands.
Direct mode also clears them when joint feedback is lost or reduced-speed safety
inhibits that mode. Commands arriving while inhibited are discarded. Recovery
requires a source command timestamped at or after readiness returns; the supervisor
cannot resume an old command by assigning it a new output timestamp. This applies
to manual velocity, autonomous velocity, and autonomous direct joint commands.
Independent STO/enable known flags report mapping limitations; the supplied
FlexiSoft configuration cannot confirm those physical signals separately.

## Modes

- `MANUAL`: requires `manual/state` to report a fresh connected joystick and active source; selects
  `manual/cmd_vel`.
  The USB profile does not require a held button. Any optional teleop deadman is
  included in `command_active`; `deadman_pressed` alone is telemetry, not a supervisor gate.
- `AUTO_VELOCITY`: selects external `autonomy/cmd_vel`.
- `AUTO_DIRECT`: selects external `autonomy/joint_setpoints` whenever mode 2 is requested, without an extra
  permission parameter. Because joint-space commands cannot be safely rescaled as vehicle
  motion, this mode is inhibited whenever the safety system requests warning-field or override reduced speed.

Velocity modes publish supervised `cmd_vel` for `mobotic_kinematics`. Direct mode republishes validated and bounded
`sensor_msgs/msg/JointState` commands on `supervisor/joint_setpoints` for `mobotic_driver`. It does not publish velocity
commands while active, avoiding a second kinematics command source.

Direct steering positions must convert to signed 32-bit encoder ticks. Configure
`direct_steering_encoder_resolutions` in `expected_steering_joint_names` order,
matching the driver's `drives.resolutions` (currently 4096 ticks/revolution for
each steering drive). Invalid configuration prevents startup. An invalid autonomy
joint command clears that source's cached command and produces a stop on the next
control cycle if selected. Measured steering positions used for stop/hold commands
must also be representable. Valid multi-turn angles are preserved, not wrapped;
these checks do not establish mechanical steering travel limits. The driver
independently validates both internal motion inputs before any CAN target write.

All three modes can be requested through `vehicle/set_mode` (`mobotic_interfaces/srv/SetVehicleMode`).
The timestamped `vehicle/mode_request` topic accepts only MANUAL/AUTO_VELOCITY;
AUTO_DIRECT requests on it are rejected. Authoritative state is published on `vehicle/mode_state`.
The joystick publishes semantic requests on the same topic; it cannot select a
driver source directly. The removed `allow_auto_direct` parameter is no longer
required. Selection does not bypass battery/safety/feedback/command validation.

```bash
# 0=MANUAL, 1=AUTO_VELOCITY, 2=AUTO_DIRECT
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode "{requested_mode: 2}"
ros2 topic echo /moboterra/vehicle/mode_state
```

## Verified mode transitions

During a transition, candidate manual and autonomous commands are blocked.
Reselecting the current mode also stops and verifies standstill. A duplicate
request for an already-pending mode does not restart the stop; a conflicting
request is rejected until completion or explicit retry after timeout.
Velocity modes ramp the last supervised `cmd_vel` to zero using the configured
acceleration limits. Leaving `AUTO_DIRECT` sends zero joint velocity/current
while holding the measured steering positions through `supervisor/joint_setpoints`.
Safety, battery, or wheel readiness loss causes an immediate stop.

The supervisor requires fresh `agv_vel` in `output_frame_id` and complete
`joint_states` feedback timestamped after the request. Planar speed must be at
most `standstill_linear_velocity` (0.02 m/s), angular speed at most
`standstill_angular_velocity` (0.02 rad/s), and every steering and traction
joint speed at most `standstill_joint_velocity` (0.1 rad/s).
Both feedback streams must keep updating below those limits for
`mode_transition_stop_duration` (0.25 s). Movement or invalid/stale feedback
resets the confirmation period. Full odometry is not required.

`mode_transition_timeout` defaults to 10 s. On timeout the old `current_mode`
and requested mode remain visible, `transition_in_progress` stays true, and
`motion_permitted` stays false. Motion remains blocked until a new valid mode
request explicitly retries the transition, including a request for the old mode.
After a successful switch, buffered commands are cleared and a new command
timestamped at or after completion is required before motion can resume.
Exactly one `current_mode` is authoritative. The driver subscribes to its state,
selects only kinematics for modes 0/1 or direct supervisor setpoints for mode 2,
and rejects delayed commands predating transition/recovery boundaries. During
stopping, only the outgoing mode's stop/ramp path is selected. Missing/stale mode
state independently stops motion. Continuous fresh commands may resume after a
successful switch; this does not add a separate operator start confirmation.

The driver executes the supervisor's global enable/disable request; the hardware
releases brakes on enable and engages them on disable. The supervisor does not
replace the hardware safety controller or control battery contactors.

The driver's independent management lease stops/disables on supervisor loss
(default 0.3 s) even if joint setpoints continue. Missing required CAN feedback
also stops all-unit motion without altering the safety/battery enable rule.
See the [interface contract](../../docs/supervisor_interfaces.md) and
[hardware loss tests](../../docs/hardware_monitoring.md).
