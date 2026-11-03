# mobotic_supervisor

MoboTerra's existing supervisor owns the vehicle mode state machine, source arbitration, controlled transitions and fault recovery. No separate vehicle controller is introduced.

The public contract is documented in [Supervisor interface contract](../../docs/supervisor_interfaces.md). Its names are relative to the robot namespace, normally `/moboterra`.

- MANUAL (0) selects `operator/cmd_vel` and fresh, rearmed `operator/state`.
- AUTONOMOUS (1) selects `autonomy/cmd_vel`.
- KINEMATICOFF (2) selects `autonomy/joint_setpoints`; velocity candidates have no motion effect.
- SAFETY (3) and ERROR (4) are automatic. ERROR has priority; both flags remain visible.
- UNKNOWN (255) represents initial monitoring/readiness that has not been established.

Select 0/1/2 with `vehicle/set_mode` (SetVehicleMode). `operator/mode_request` accepts 0/1 only.

The supervisor publishes `vehicle/mode`, `vehicle/cmd_vel` for kinematics, the final `wheel_modules/command` array, and internal `wheel_modules/operation_command` management transactions. Kinematics returns a correlated candidate; it never publishes directly to the driver's motion input.

Candidate validation and final publication run in the candidate receive callback,
without waiting for another 20 ms control tick. The extra ROS hop has unmeasured
transport/scheduling overhead. See [command fields, EC comparison and timing](../../docs/control_path_and_latency.md);
configured periods are not measured end-to-end response or stopping guarantees.

Transitions stop and verify fresh body/all-joint standstill for 0.25 s before changing sources. A 10 s timeout latches ERROR. Safety recovery requires a fresh command; manual control also requires returning the controls to neutral. A release/new press is required only if a physical deadman is configured. The two-module PAIL profile has no deadman and requires no held button. Actual faults require `wheel_modules/clear_errors` (Trigger). Fresh sustained standstill or individual native disabled states admit fault clearing; external interlocks and neutral controls remain mandatory. Fresh matching driver transaction IDs permit zero-target initialization/enable while ERROR remains latched. Fault-clearance acknowledgements, healthy enabled feedback and a new sustained standstill proof are required before latch clearance. Reset acceptance is not proof of recovery. See [the recovery sequence](../../docs/drive_startup.md).

Default command expiry is 250 ms, the control cycle 20 ms, startup grace 10 s and reset timeout 20 s (covering the 15 s drive startup attempt). Policies, module names and limits come from `mobotic_config`. Monitoring loss, actual drive/BMS faults and required component health are root fault inputs; protective stops and low SOC are not themselves system errors.

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
invalid input/frame/timestamp, a hardware interlock (including B2 drive disable,
startup or latched faults), or a sample acquired before the last readiness/mode
change. A hardware interlock does not imply that the PAIL input is stale.
Repeated identical reasons are logged at most once every two seconds; a changed
reason is reported immediately. Inspect `vehicle/mode` for current interlocks,
`reason_codes` and reset progress. Clearing a hardware blocker still requires a
new input sample; command expiry and recovery gates remain enforced.

Outside an authorized MANUAL RP2/D5 bypass, both
`scanner/front_left/safety_zone_status` and
`scanner/rear_right/safety_zone_status` must be fresh, known and communicating,
including on the two-module `hardwired_sto` profile. The most restrictive field
wins across the scanners:

| Fields | Supervisor behavior |
|:--|:--|
| Any red protective field active | Publish inhibited driver authority, zero body velocity and final wheel HOLD targets from the scanner callback; invalidate pending motion and require a fresh command after recovery |
| Any yellow warning field active, no red | Cap planar and final wheel peripheral commands at 0.3 m/s; slower requested speeds remain slower |
| Both scanners clear | Remove the scanner cap; normal platform limits, source arbitration and other interlocks apply |
| Either scanner missing, invalid or stale | Inhibit motion; after startup the monitoring loss latches `SCANNER_MONITOR_LOST:<location>` and uses normal explicit fault recovery |

Yellow warning zones and Safety Disabled/override share the same absolute
**0.3 m/s maximum**, for two and four wheel modules. Warning status may come
from either scanner, FlexiSoft warning fields or FlexiSoft I/O. Reduced-speed
safety status may come from FlexiSoft `safety_override_active`, or the existing
commissioned RP2/D5 policy for either layout. This speed rule does not authorize a new
scanner bypass or relax any mode-selection, red-zone, STO, E-stop, battery or
fault interlock.

The body-command limit uses `hypot(vx, vy)`, including reverse and crab, with a
proportional angular limit. The final supervisor output also caps every wheel's
peripheral speed (`abs(traction_velocity) * wheel_radius`) at 0.3 m/s. Wheel
commands are scaled together to retain direction and relative wheel speeds;
this covers pure rotation, combined translation/rotation, direct wheel commands
and controlled stops. A lower `warning_speed_limit` is honored, while a higher
overlay cannot raise the project ceiling. Clear fields and no override restore
normal platform limits; normal low-speed commands remain low-speed.

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

B2 uses `wheel_modules/toggle_drives`: it latches drive disable or cancels active
reset/startup, then a new press requests supervised zero-target recovery. The
operator adapter never clears or restarts the receiver. Both AUTO modes require
a fresh RP2 Safety Enabled selector when the commissioned override is configured.
A faulted return to MANUAL requires fresh native disabled-drive proof and no
observed movement; it preserves fault latches until explicit recovery completes.

RP2/D5 scanner bypass is supported on both two- and four-module layouts with
verified REMdevice mapping (`scanner_override: {signal: D5, active_high: true}`).
D5 high permits only reduced-speed MANUAL (0.3 m/s); both AUTO modes require
D5 low with fresh valid selector/operator samples. D5 low, stale or invalid input
restores scanner checks, and either selector edge clears buffered motion.
Four-module operation still requires fresh FlexiSoft safety and I/O messages.
Only scanner protective-field/OSSD conditions are exempted; FlexiSoft E-stop,
STO, safety-enable, communication/system readiness, operator STOP, battery and
all drive checks remain active. An unexplained PLC motion prohibition remains
blocked. Bypass changes ROS scanner policy, not physical safety-controller wiring.

SOC below `minimum_battery_percentage` (default 0.2 / 20%) is warning-only for
all layouts: it does not inhibit startup, drive enable, motion or B2 recovery.
The compatibility parameter name is retained, but now sets the warning threshold.
`vehicle/mode` includes advisory `LOW_BATTERY_SOC` and warning text without
setting `error_active` or latching a fault. `/diagnostics` reports WARN, and the
readiness checker lists a warning instead of a blocker. Missing/stale/invalid
battery measurements, HV disconnection, charging interlocks and BMS faults
retain their existing blocking behavior; physical BMS protection is unchanged.

When a fresh RP2/D5 sample says Safety Enabled but fresh FlexiSoft telemetry
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
