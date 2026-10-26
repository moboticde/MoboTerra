# mobotic_supervisor

MoboTerra's existing supervisor owns the vehicle mode state machine, source arbitration, controlled transitions and fault recovery. No separate vehicle controller is introduced.

The public contract is documented in [Supervisor interface contract](../../docs/supervisor_interfaces.md). Its names are relative to the robot namespace, normally `/moboterra`.

- MANUAL (0) selects `operator/cmd_vel` and fresh, rearmed `operator/state`.
- AUTONOMOUS (1) selects `autonomy/cmd_vel`.
- KINEMATICOFF (2) selects `autonomy/joint_setpoints`; velocity candidates have no motion effect.
- SAFETY (3) and ERROR (4) are automatic. ERROR has priority; both flags remain visible.
- UNKNOWN (255) represents initial monitoring/readiness that has not been established.

Select 0/1/2 with `vehicle/set_mode` (SetVehicleMode). `operator/mode_request` accepts 0/1 only. Legacy AUTO_VELOCITY/AUTO_DIRECT constants remain aliases, not extra modes.

The supervisor publishes `vehicle/mode`, `vehicle/cmd_vel` for kinematics, the final `wheel_modules/command` array, and internal `wheel_modules/operation_command` management transactions. Kinematics returns a correlated candidate; it never publishes directly to the driver's motion input.

Candidate validation and final publication run in the candidate receive callback,
without waiting for another 20 ms control tick. The extra ROS hop has unmeasured
transport/scheduling overhead. See [command fields, EC comparison and timing](../../docs/control_path_and_latency.md);
configured periods are not measured end-to-end response or stopping guarantees.

Transitions stop and verify fresh body/all-joint standstill for 0.25 s before changing sources. A 10 s timeout latches ERROR. Safety recovery requires a fresh command; manual control also requires returning the controls to neutral. A release/new press is required only if a physical deadman is configured. The two-module PAIL profile has no deadman and requires no held button. Actual faults require `wheel_modules/clear_errors` (Trigger). Fresh sustained standstill or individual native disabled states admit fault clearing; external interlocks and neutral controls remain mandatory. Fresh driver acknowledgements permit zero-target enable, followed by healthy enabled feedback and a new sustained standstill proof before latch clearance. Reset acceptance is not proof of recovery. See [the recovery sequence](../../docs/drive_startup.md).

Default command expiry is 250 ms, the control cycle 20 ms, startup grace 10 s and reset timeout 5 s. Policies, module names and limits come from `mobotic_config`. Monitoring loss, actual drive/BMS faults and required component health are root fault inputs; protective stops and low SOC are not themselves system errors.

Both `scanner/front_left/safety_zone_status` and
`scanner/rear_right/safety_zone_status` must be fresh, known and communicating,
including on the two-module `hardwired_sto` profile. The most restrictive field
wins across the scanners:

| Fields | Supervisor behavior |
|:--|:--|
| Any red protective field active | Publish inhibited driver authority, zero body velocity and final wheel HOLD targets from the scanner callback; invalidate pending motion and require a fresh command after recovery |
| Any yellow warning field active, no red | Ramp toward an absolute planar speed cap of `warning_speed_limit: 0.3` m/s using `max_acceleration_linear`; slower requested speeds remain slower |
| Both scanners clear | Remove the scanner cap; normal platform limits, source arbitration and other interlocks apply |
| Either scanner missing, invalid or stale | Inhibit motion; after startup the monitoring loss latches `SCANNER_MONITOR_LOST:<location>` and uses normal explicit fault recovery |

Yellow caps the magnitude `hypot(vx, vy)`, including reverse and crab motion,
and proportionally limits the angular target. It does not scale acceleration
down to 30%, and it does not step the current command down to the cap: the
deceleration ramp can temporarily remain above 0.3 m/s. Red bypasses that ramp.
When fields clear, acceleration resumes toward the requested speed.

KINEMATICOFF/direct wheel commands also honor yellow: traction targets are
scaled proportionally so each wheel's peripheral speed is at most 0.3 m/s,
then slewed with the platform linear acceleration and configured wheel radius.
This conservatively limits rotation too. Red stops direct commands immediately.
Safety override retains its separate existing restrictions and cannot bypass a
red field reported directly by a scanner. In-flight kinematics from a previous
scanner speed policy are discarded without changing the driver authority on
yellow, preserving the ramp. Red inhibition also interrupts an active controlled
mode transition at the driver. No drive-enable, battery, operator or fault
checks are bypassed when the fields clear.

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
