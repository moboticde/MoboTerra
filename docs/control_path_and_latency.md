# Wheel command path, EC comparison and latency

Source review: 23 October 2026. This describes the local MoboTerra source and
the two-module `platform-front-right-rear-left.yaml` profile. Timing values are
configured periods, not measurements of an installed robot. Policy overlays
can change them.

## Command and feedback routing

All topic names are relative to the robot namespace, normally `/moboterra`.
A topic is a named channel; its message type defines the payload.

```text
operator/cmd_vel or autonomy/cmd_vel
                   |
                   v
               supervisor
                   | vehicle/cmd_vel (TwistStamped)
                   v
               kinematics <--- joint_states (JointState) --- driver
                   |
                   | kinematics/joint_setpoints (JointState candidate)
                   v
               supervisor
                   | wheel_modules/command (WheelModuleCommandArray)
                   v
                 driver ---> CAN ---> controllers
```

In KINEMATICOFF, `autonomy/joint_setpoints` goes to the supervisor and uses the
same final `wheel_modules/command` input. It bypasses inverse kinematics for
commands; measured feedback and monitoring still operate.

Kinematics does not initialize or enable drives. The supervisor uses
`wheel_modules/operation_command` for enable/disable and reset transactions.
Startup can authorize zero-target enable before motion feedback is available;
fresh complete feedback is required before accepting motion.

## Comparison with romb_ws_ec

The comparison uses `romb_ws_ec/romb_ws_ec/src/terra_spiro`, including its
`mobotic_launch/launch/terraspiro.launch.py`. Standalone launches or other
launch files can use different namespaces/remappings.

| Connection or behavior | romb_ws_ec | MoboTerra |
| --- | --- | --- |
| Velocity input to kinematics | `cmd_vel`, remapped to `cmd_vel_supervised` in the main launch; `TwistStamped` | `vehicle/cmd_vel`; `TwistStamped` |
| Kinematics output | `joint_setpoints`; `JointState`, directly to driver | `kinematics/joint_setpoints`; `JointState`, back to supervisor |
| Driver motion input | `joint_setpoints`; `JointState` | `wheel_modules/command`; `mobotic_interfaces/msg/WheelModuleCommandArray`, supervisor only |
| Driver feedback to kinematics | `joint_states`; `JointState` | `joint_states`; `JointState` |
| Candidate publication trigger | Computes on velocity input, then republishes the stored targets on feedback while enabled | Computes/publishes on valid velocity input when cached feedback is fresh |
| Feedback traction position | Published as zero | Tracked controller rotation, subject to position validity |
| `JointState.effort` | Motor current in feedback; optional current-mode command values | Empty; current is reported in `wheel_modules/status`; traction commands use velocity mode |
| Enable/reset interface | Separate driver services | Supervisor-owned management messages and verified reset transaction |

Both command paths carry steering target angle [rad], steering profile speed
[rad/s] and signed traction wheel speed [rad/s]. Both associate joints by name;
their wire contracts are not interchangeable by topic renaming alone.

## Final wheel command message

`wheel_modules/command` uses
[`mobotic_interfaces/msg/WheelModuleCommandArray`](../src/mobotic_interfaces/msg/WheelModuleCommandArray.msg).

| Field | Meaning |
| --- | --- |
| `header` | Final-command issue stamp; the driver rejects stale or out-of-order commands |
| `source_stamp` | Source stamp retained for motion freshness; stop/hold paths use their supervisor-issued stamp |
| `supervisor_instance` | Identifies the supervisor instance |
| `authority_generation` | Identifies the current authority period and invalidates earlier commands |
| `selected_mode` | Selected MANUAL=0, AUTONOMOUS=1 or KINEMATICOFF=2 mode |
| `purpose` | MOTION=0, CONTROLLED_STOP=1 or HOLD=2 |
| `modules[]` | Exactly one named target record per configured module |

Each [`WheelModuleCommand`](../src/mobotic_interfaces/msg/WheelModuleCommand.msg)
record contains:

| Field | Unit and meaning |
| --- | --- |
| `name` | Module name: `front_right` or `rear_left` in the reviewed two-module profile |
| `steering_position` | Target steering angle [rad] |
| `steering_profile_velocity` | Nonnegative steering movement profile speed [rad/s] |
| `traction_velocity` | Signed wheel/output rotation speed [rad/s], not motor rpm or body m/s |

The default four-module profile has four records; the two-module profile has
two. These are motion targets, not measured positions, current commands or
enable requests. `wheel_modules/operation_command` separately carries the
supervisor identity/generation, timestamp, enable flag and reset ID.

CONTROLLED_STOP only reduces existing traction magnitude without reversing;
HOLD requests zero traction. Both hold actual steering. The driver independently
checks completeness, finite values, limits, mode/authority, feedback freshness,
fault/reset/stop state and source age. It chooses its own measured steering hold
for stop/hold commands.

## Latency and reaction time

The return through the supervisor adds one ROS message delivery and callback
compared with sending kinematics output directly to the driver. It does not
add another fixed 20 ms control-cycle wait: the supervisor correlates, validates
and emits the final command inside `_kinematics_candidate_callback`.

| Stage | Configured behavior |
| --- | --- |
| Operator node publication | 20 ms policy period, before the supervisor input boundary |
| Supervisor velocity selection/limiting | 20 ms control period (50 Hz) |
| Kinematics calculation | Runs in the velocity receive callback with fresh joint feedback |
| Supervisor candidate approval | Runs in the candidate receive callback; no separate periodic wait |
| Traction CAN PDO transmission | Driver stores the target, then sends it on the 20 ms PDO cycle |
| Steering command transmission | Queued profile-speed/position writes; CAN/SDO response and queue time apply |

Under ideal scheduling, the supervisor and traction-PDO phases contribute
approximately 0–40 ms of waiting from a command already received by the
supervisor to traction CAN transmission. This is only the contribution of those
two periodic stages. It is neither a measured end-to-end latency nor an upper
bound. PAIL acquisition/radio timing, operator publication, ROS transport,
executor contention, CPU load, CAN arbitration/queues, controller response and
mechanics add time. The EC feedback-triggered command publication also introduces
phase dependence; source inspection alone cannot establish which robot reacts
faster.

The reviewed profile limits supervisor linear acceleration to 0.5 m/s². A
single such ramp needs approximately two seconds to change from 0 to 1 m/s,
assuming a sustained request and no other limiting stages. Operator shaping,
controller ramps, steering alignment and mechanics can further affect actual
response. This ramping is separate from message latency. Mode changes also
intentionally stop and require 0.25 s of fresh measured standstill; that proof
duration is not a delay added to every driving command.

The 100 ms driver command watchdog, 250 ms motion-source age limit and 300 ms
supervisor lease are expiry guards, not normal transmission delays or guaranteed
physical stopping times.

Keep one final supervisor-owned motion input. Adding a direct executable
`joint_setpoints` path would need equivalent current-mode, source-correlation,
transition/reset and authority checks in the driver. A parallel path would
undermine the present final-approval contract. Measure first; reduce demonstrated
transport, scheduling or CAN bottlenecks while preserving that contract.

## Target-host timing validation

Actual latency remains unmeasured. On the running target, passively observe
operator/autonomy input, `vehicle/cmd_vel`, `kinematics/joint_setpoints`,
`wheel_modules/command`, `vehicle/mode`, `can_tx`, `can_rx`, `joint_states` and
`vehicle/measured_velocity`. Compare normal and representative CPU/CAN load.
Use callback/transport tracing to distinguish execution time from queue delay;
topic frequency alone is not an end-to-end latency measurement.

The kinematics candidate preserves `vehicle/cmd_vel.header.stamp`, allowing it
to be matched to the request. Final `header.stamp` records supervisor issue
time and `source_stamp` preserves motion-input age. Feedback preserves motion
acquisition time. Bag receipt times and these fields provide useful milestones,
but do not independently measure controller/mechanical response or isolate every
callback interval. Record typical, high-percentile and maximum observed delay,
command gaps, watchdog activations and the selected policy/profile. Evaluate
physical reaction only during the existing secured hardware acceptance process.

Implementation references: [kinematics](../src/mobotic_kinematics/src/mobotic_kinematics.cpp),
[supervisor](../src/mobotic_supervisor/mobotic_supervisor/supervisor.py),
[driver](../src/mobotic_driver/src/mobotic_driver.cpp),
[supervisor policy](../src/mobotic_config/config/supervisor.yaml),
[driver policy](../src/mobotic_config/config/driver.yaml) and
[two-module profile](../src/mobotic_config/config/platform-front-right-rear-left.yaml).
