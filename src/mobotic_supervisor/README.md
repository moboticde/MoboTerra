# mobotic_supervisor

MoboTerra's existing supervisor owns the vehicle mode state machine, source arbitration, controlled transitions and fault recovery. No separate vehicle controller is introduced.

The public contract is documented in [Supervisor interface contract](../../docs/supervisor_interfaces.md). Its names are relative to the robot namespace, normally `/moboterra`.

- MANUAL (0) selects `operator/cmd_vel` and fresh, rearmed `operator/state`.
- AUTONOMOUS (1) selects `autonomy/cmd_vel`.
- KINEMATICOFF (2) selects steering position/profile speed and traction motor current from `autonomy/joint_setpoints`; body-velocity candidates have no motion effect.
- SAFETY (3) and ERROR (4) are automatic. ERROR has priority; both flags remain visible.
- UNKNOWN (255) represents initial monitoring/readiness that has not been established.

Select 0/1/2 with `vehicle/set_mode` (SetVehicleMode). `operator/mode_request` accepts 0/1 only.

KINEMATICOFF traction uses current control. Its JointState candidate contains
every configured joint with full finite `position`, `velocity` and `effort`
arrays. Steering uses position [rad], nonnegative profile speed [rad/s] and zero
effort. Traction uses zero velocity and signed current [A] in `effort`; traction
position is ignored. This is a project-specific command convention: effort here
is electrical current, not mechanical torque. Feedback JointState.effort stays
empty; measured current remains in wheel-module status.

After fresh sustained standstill, the supervisor authorizes a disabled,
zero-target traction mode transaction and waits for controller acknowledgement
before selecting the new mode. miControl uses native mode 2 / `0x3200` for
current, and mode 3 / `0x3300` for normal velocity control. Returning to MANUAL
or AUTONOMOUS restores velocity mode through the same transaction. Both require
fresh commands after completion. A mode-write rejection fails startup and keeps
motion inhibited until explicit recovery. Native encoder references are retained.

Current mode is verified only for miControl traction. Profiles containing ELMO
traction reject KINEMATICOFF selection/initialization; their velocity modes remain
available. No miControl register or guessed DS402 current scaling is sent to ELMO.
The prepared four-module miControl profile supports current mode with the
configured +/-4 A limit, further bounded by each drive's raw mA limits.

Current cannot enforce a reduced-speed cap, so warning/reduced-speed conditions
inhibit KINEMATICOFF current output and request zero current. The driver also
monitors measured wheel velocity: current-mode overspeed stops/disables drives,
latches fault `0x7FFF0001`, and requires an explicit supervised reset. Zero
current removes the commanded motor current; it is not velocity braking or a
standstill guarantee. Mode transitions continue to require measured standstill.

The default four-module profile selects `safety.telemetry_required: false`.
FlexiSoft Ethernet feedback is monitoring only; hardware STO is operated by the
physical safety circuit. FlexiSoft messages and their absence do not govern
enable, modes, recovery, speed limits or fault latches in this role. Scanner
fields, operator STOP, battery readiness, drive feedback/faults, known active
drive STO and watchdogs still apply. `safety.telemetry_required: true` restores
the additional FlexiSoft software interlocks. Missing fields in older profiles
default to that required-telemetry behavior.

The supervisor publishes `vehicle/mode`, `vehicle/cmd_vel` for kinematics, the final `wheel_modules/command` array, and internal `wheel_modules/operation_command` management transactions. Kinematics returns a correlated candidate; it never publishes directly to the driver's motion input.

Candidate validation and final publication run in the candidate receive callback,
without waiting for another 20 ms control tick. The extra ROS hop has unmeasured
transport/scheduling overhead. See [command fields, EC comparison and timing](../../docs/control_path_and_latency.md);
configured periods are not measured end-to-end response or stopping guarantees.

Transitions stop and verify fresh body/all-joint standstill for 0.25 s before changing sources. A 10 s timeout latches ERROR. Safety recovery requires a fresh command; manual control also requires returning the controls to neutral. Hold-to-drive input support has been removed; neutral rearm and fresh commands remain required. Actual faults require `wheel_modules/clear_errors` (Trigger). Fresh sustained standstill or individual native disabled states admit fault clearing; external interlocks remain mandatory. Neutral controls are required to start
recovery. Stick movement after acceptance is ignored while zero-target recovery
runs; it does not request drive disable. Fresh neutral rearm is still required
after recovery completes. Fresh matching driver transaction IDs permit zero-target initialization/enable while ERROR remains latched. Fault-clearance acknowledgements, healthy enabled feedback and a new sustained standstill proof are required before latch clearance. Reset acceptance is not proof of recovery. See [the recovery sequence](../../docs/drive_startup.md).

Default command expiry is 250 ms, the control cycle 20 ms, startup grace 10 s and reset timeout 20 s (covering the 15 s drive startup attempt). Policies, module names and limits come from `mobotic_config`. Monitoring loss, actual drive/BMS faults and required component health are root fault inputs; protective stops and low SOC are not themselves system errors.

Fresh connected neutral operator evidence or an explicit fresh zero manual
command cancels manual motion without a software deceleration ramp. It also replaces older in-flight kinematics results
with zero traction and zero steering-profile holds at measured positions, so a
late candidate cannot resume a released move. Neutral evidence must be at least
as new as the candidate's source input; newer joystick motion remains eligible.
Drive deceleration and all existing feedback/authority checks still apply.

When `motion_limits.steering_return_home_on_neutral` is enabled, a fresh zero
manual command plus fresh connected all-controls-neutral evidence instead
authorizes a zero-traction return to nearest full-turn home. The first batch
stops all profiles; steering waits for a newer feedback sample with every
traction joint at standstill. This remains a normal permitted MANUAL motion,
so safety, source, feedback, mode-transition and reset gates still apply.
Interlock HOLD commands cannot home. Stick movement cancels the return.

For component health, `ComponentHealthArray.header` is the producer's report
time; each component's header remains its input acquisition time. Both report
age and local receipt age must fit `health_timeout` (300 ms). Kinematics and
odometry input age separately uses `health_input_timeout`, derived from the
producer's `diagnostics.odometry_timeout` (500 ms). Host health retains its
300 ms bound. Invalid monitoring or a reported fault still inhibits immediately.
Old/replayed reports cannot renew the watchdog, and fresh reports cannot keep
an expired acquisition alive. Drive, command and odometry integration deadlines
are independent of these component-health checks.

`Manual velocity inhibited: ...` identifies why a PAIL/USB command was refused:
invalid input/frame/timestamp, a hardware interlock (including the B2 enable wait,
startup or latched faults), or a sample acquired before the last readiness/mode
change. A hardware interlock does not imply that the PAIL input is stale.
Repeated identical reasons are logged at most once every two seconds; a changed
reason is reported immediately. Inspect `vehicle/mode` for current interlocks,
`reason_codes` and reset progress. Clearing a hardware blocker still requires a
new B2 enable transaction and fresh input; command expiry and recovery gates
remain enforced. Startup also waits for B2. The supervisor stays running during
an error or missing telemetry and publishes `WAITING: <cause>. Action: <remedy>`
with instructions such as restart the joystick/receiver, disconnect the charger,
or check battery CAN/scanner Ethernet. The console reports changes immediately
and repeats the same waiting instruction at most every five seconds.
`B2_ENABLE_REQUIRED` identifies the wait in `vehicle/mode.reason_codes`.
See the [operator recovery guide](../../docs/drive_startup.md) for the action
table and the complete B2 sequence. A fault during recovery cancels the attempt
and waits for a new press after correction; it cannot resume automatically.

Outside an authorized MANUAL RP2/D5 bypass, both
`scanner/front_left/safety_zone_status` and
`scanner/rear_right/safety_zone_status` must be fresh, known and communicating,
on the supported four-module platform. The most restrictive field
wins across the scanners:

| Fields | Supervisor behavior |
|:--|:--|
| Any red protective field active | Publish inhibited driver authority, zero body velocity and final wheel HOLD targets from the scanner callback; invalidate pending motion and require a fresh command after recovery |
| Any yellow warning field active, no red | Cap planar and final wheel peripheral commands at 0.3 m/s; slower requested speeds remain slower |
| Both scanners clear | Remove the scanner cap; normal platform limits, source arbitration and other interlocks apply |
| Either scanner missing, invalid or stale | Inhibit motion; after startup the monitoring loss latches `SCANNER_MONITOR_LOST:<location>` and uses normal explicit fault recovery |

Yellow warning zones and Safety Disabled/override share the same absolute
**0.3 m/s maximum**, for two and four wheel modules. Warning status may come
from either scanner and, when FlexiSoft telemetry is required for control,
FlexiSoft warning fields or FlexiSoft I/O. Reduced-speed safety status may come
from FlexiSoft `safety_override_active` in that control role, or the existing
commissioned RP2/D5 policy for either layout. This speed rule does not authorize a new
scanner bypass or relax any mode-selection, red-zone, STO, E-stop, battery or
fault interlock.

`vehicle/mode.reason_codes` and `status_message` identify every source of a
reduced-speed cap: `SCANNER_WARNING:<location>`, `SAFETY_DISABLED_RP2`,
`FLEXISOFT_WARNING`, `FLEXISOFT_IO_WARNING` or `SAFETY_DISABLED_FLEXISOFT`.
The supervisor logs changes to this policy. Both scanners clear and Safety
Enabled remove this cap; enabling scanner monitoring alone never reduces speed.

The body-command limit uses `hypot(vx, vy)`, including reverse and crab, with a
proportional angular limit. For the manual steering policy, either body limit
uniformly scales forward, lateral and yaw velocity, preserving the curvature or
crab direction of the velocity being capped. This applies to target limits and
the immediate output cap when a reduced-speed condition begins. The existing
acceleration ramps still govern changes between requested velocities.
The final supervisor output also caps every wheel's
peripheral speed (`abs(traction_velocity) * wheel_radius`) at 0.3 m/s. Wheel
commands are scaled together to retain direction and relative wheel speeds;
this covers pure rotation, combined translation/rotation, velocity wheel commands
and controlled stops. A lower `warning_speed_limit` is honored, while a higher
overlay cannot raise the project ceiling. Clear fields and no override restore
normal platform limits; normal low-speed commands remain low-speed.
Current-mode KINEMATICOFF is inhibited rather than scaled by this velocity rule.

Published commands are capped as soon as the condition is processed, even if
the previous command was faster. Actual physical slowing takes time and follows
the drive's configured deceleration and the robot's dynamics. Red still commands
zero immediately. In-flight kinematics from an earlier scanner policy are
invalidated. `override_speed_scale` is accepted for old configuration files but
no longer determines speed or acceleration: this policy is an absolute m/s cap,
not a percentage of the platform's maximum speed.

`warning_speed_limit` replaces the old dimensionless `warning_speed_scale`
parameter; update custom supervisor overlays accordingly. The limits operate on
commands, not measured stopping distance. ROS callback latency, Ethernet delay,
drive braking and scanner path mapping still require hardware commissioning.

```bash
ros2 topic echo /moboterra/operator/state
ros2 topic echo /moboterra/vehicle/mode
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode "{requested_mode: 2}"
ros2 service call /moboterra/wheel_modules/clear_errors std_srvs/srv/Trigger "{}"
```

Mode/reset calls affect the robot; use a secured platform. ROS supervision is not safety-rated. The physical safety chain remains independent, and this update is not hardware-commissioned.

B2 uses `wheel_modules/enable_drives`, like Logitech X in the example project:
it requests initialization/enable while disabled, and leaves pending or completed
enable unchanged on subsequent presses. It cannot cancel reset/startup or disable
drives. The explicit `wheel_modules/toggle_drives` service remains separate. The
operator adapter never clears or restarts the receiver. Both AUTO modes require
a fresh RP2 Safety Enabled selector when the commissioned override is configured.
A faulted return to MANUAL requires fresh native disabled-drive proof and no
observed movement; it preserves fault latches until explicit recovery completes.

RP2/D5 scanner bypass is supported on both two- and four-module layouts with
verified REMdevice mapping (`scanner_override: {signal: D5, active_high: true}`).
D5 high permits only reduced-speed MANUAL (0.3 m/s); both AUTO modes require
D5 low with fresh valid selector/operator samples. D5 low, stale or invalid input
restores scanner checks, and either selector edge clears buffered motion.
When `safety.telemetry_required: true`, fresh FlexiSoft safety and I/O messages
remain required. Only scanner protective-field/OSSD conditions are exempted;
FlexiSoft E-stop, STO, safety-enable and communication/system readiness still
apply in that role. With monitoring-only telemetry, those Ethernet values do
not control ROS. Operator STOP, battery and drive checks apply in either role.
Bypass changes ROS scanner policy, not physical safety-controller wiring.

SOC below `minimum_battery_percentage` (default 0.2 / 20%) is warning-only for
all layouts: it does not inhibit startup, drive enable, motion or B2 recovery.
The compatibility parameter name is retained, but now sets the warning threshold.
`vehicle/mode` includes advisory `LOW_BATTERY_SOC` and warning text without
setting `error_active` or latching a fault. `/diagnostics` reports WARN, and the
readiness checker lists a warning instead of a blocker. Missing/stale/invalid
battery measurements, HV disconnection, charging interlocks and BMS faults
retain their existing blocking behavior; physical BMS protection is unchanged.

When FlexiSoft telemetry is required for control and a fresh RP2/D5 sample says
Safety Enabled but fresh FlexiSoft telemetry
reports safety override active, all motion sources (MANUAL, AUTONOMOUS and
KINEMATICOFF), drive enable and B2 recovery are inhibited. The supervisor
immediately publishes stop authority and zero targets from the receiving callback.
`vehicle/mode` reports SAFETY with `SAFETY_SELECTOR_MISMATCH`; this disagreement
alone is not a latched ERROR, so normal signal propagation does not require a
fault reset. It remains blocked while present. Old commands are discarded and
fresh commands/manual rearming are required after agreement is restored. B2
still permits disabling; it cannot enable through this interlock. Missing or stale
RP2 input cannot release the block while PLC override remains active. FlexiSoft
is read-only: ROS blocks its own commands and does not modify PLC logic or STO.


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

## Readiness evaluation and drive management

The driver owns per-field CAN acquisition freshness. `_driver_status_gate`
checks delivery age and module bindings of the ROS status message.
`_enable_gate` adds external interlocks and recovery constraints; `_base_gate`
adds initialized/enabled modules and consumes the driver's `feedback_fresh`
result. It does not repeat status delivery or topology validation.

Each control tick evaluates motion readiness, gathers unique fault codes, applies
`_drive_enable_decision`, then publishes the resulting management command through
`_publish_drive_operation`. Recovery can change authority/reset state, so motion
readiness is evaluated again after recovery and reused for source selection.
Callbacks and services evaluate current state; no readiness cache persists between
calls. Distinct drive, transport and dependent-component fault codes remain
visible even when they describe consequences of the same missing feedback.

| Condition | Existing policy |
| --- | --- |
| No fresh motion command, with healthy connected source and hardware | Zero motion; enable remains independently permitted |
| Loss of established hardware/component readiness | Stop and withdraw enable; B2 recovery remains required |
| Driver status missing, stale or inconsistent | Enable blocked |
| Accepted reset | Only the existing acknowledged recovery transaction can authorize initialization; motion remains blocked |

The structural cleanup does not change deadlines, scanner policy, hardware STO,
reset admission or automatic recovery behavior. A stop-only versus disable policy
change requires a separate behavioral change; it must not be inferred by removing
freshness checks from one layer.
