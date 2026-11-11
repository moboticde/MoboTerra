# mobotic_driver

C++ ROS 2 driver for miControl steering and traction. Both hardware and virtual
controllers use expedited CANopen SDOs. Elmo/DS402 and drive PDO transport are
unsupported. The driver leaves installed controller ramps, encoder reference
and saved configuration intact.

## Source structure

- `communication.hpp`: miControl objects, native request encoding, validated
  SDO decoding, unit conversion and per-field acquisition ordering.
- `sdo_configuration.hpp`: acknowledged initialization writes and strict
  configuration readback. Action commands require acknowledgement only.
- `can_scheduling.hpp`: per-node SDO transactions, round-robin admission and
  the shared final-sender queue.
- `mobotic_driver.cpp`: supervisor authority, startup, polling, motion commands,
  status and diagnostics.
- `virtual_drive_protocol.hpp`: deterministic miControl SDO test model with
  separate requested and actual enable state.

## Protocol and units

| Quantity | Object | Native units / interpretation |
| --- | --- | --- |
| Fault reset / profile Update | `3000:0` | command 1 / command 5 |
| Fault register | `3001:0` | 32-bit vendor register |
| Actual status | `3002:0` | enabled bit 0, fault bit 1 |
| Operating mode | `3003:0` | current 2, velocity 3, S-velocity 5, position 7 |
| Software enable | `3004:0` | requested enable; separate from actual status |
| Current target | `3200:0` | signed mA |
| Current feedback | `3262:0` | mA; signed `3262:1` replies also accepted |
| Velocity / steering profile target | `3300:0` | motor rpm |
| S-velocity target | `3500:0` | motor rpm |
| Position feedback / target | `3762:0` / `3790:0` | configured controller position units |
| Velocity feedback | `3A04:1` | signed motor rpm |

Mode, enable, reset and targets use four-byte expedited writes (`0x23`).
`3004` readback never changes actual enabled state or refreshes status evidence.
Optional software-enable polling defaults to false.

ROS steering positions/velocities use radians and rad/s. Traction velocity is
wheel-contact m/s, converted at the CAN boundary using wheel radius and gearing.
Traction position uses `resolution * gear_ratio` counts per wheel revolution;
steering uses its configured output-position scale. Both supplied platform
profiles use 121 steering position units per output revolution and a separate
4096-count motor encoder with 16:1 gearing for traction. Current commands use A
in ROS and mA on CAN; measured current is status telemetry, not JointState effort.

## Startup and feedback

Before physical enable authorization, the driver polls native status, mode and
faults and sends its master heartbeat. Authorized startup disables the drives,
zeros targets, clears faults and selects the native mode. It reads configuration
back before enabling. Position and velocity are acquired after physical enable;
complete fresh feedback is required before motion. The virtual topic bus also
polls disabled stationary feedback so virtual startup can prove readiness.

An initialization attempt has a 15 s steady-clock deadline. Lost authority,
missing replies, failed readback or timeout stops/disables the drives. Native
status and fault polling continues after failure. A new supervised B2 reset
transaction is required to rearm. Encoder origin and controller ramps are never
rewritten. See [startup and B2 recovery](../../docs/drive_startup.md).

`traction_pdo_enabled` and `telemetry_pdo_enabled` accept only false for existing
SDO override compatibility; true is rejected. No drive PDO configuration,
NMT transaction, RPDO command or SYNC is generated. Incoming drive PDOs cannot
update feedback, freshness or availability. The final shared sender still
supports the separate operator receiver's NMT traffic.

Each validated SDO sample retains its acquisition timestamp and per-field replay
watermark. Heartbeats, acknowledgements, current and unrelated objects cannot
renew motion evidence. Physical joint publications require position/velocity
for every configured joint within `physical_sample_skew_budget` (100 ms), stamped
at the oldest required acquisition. The virtual bus uses `sample_skew_budget`
(5 ms). Traction counters unwrap with discontinuity rejection. Configuration
whose physical scan cannot fit its skew budget is rejected before transmission.

## Scheduling and guards

`paced_socket_can_sender` owns the shared 2 ms minimum gap between nonblocking
socket attempts, covering driver SDOs, operator traffic and heartbeat. Per-node
queues allow one outstanding SDO and require a matching reply. A delayed
executor gains no catch-up credit. The final queue holds at most 64 frames.
Motion/configuration writes expire from their original source timestamp after
at most 50 ms (or half the watchdog if tighter); passive reads retain the 100 ms
reply window. Stop/disable takes priority and revokes older moving/enabling
writes. Profile-zero stopping requires an acknowledged zero followed by command
5 (Update); newer motion cancels an unsent old Update.

Required fault reads drain before another motion generation. Status/mode polls
remain active in every state. Optional current/software-enable reads share a
bounded background slot and cannot replace required evidence. Existing motion
and status deadlines remain 300 ms, fault freshness 2.5 s, management lease
300 ms and command watchdog 100 ms unless configured otherwise. All mode,
lease, reset and current-mode overspeed guards remain active.

## Topics and services

Names are relative to the robot namespace, normally `/moboterra`.

| Direction | Topic | Role |
| --- | --- | --- |
| Input | `can_rx` | Acquisition-stamped CAN frames |
| Input | `wheel_modules/command` | Final motion array with supervisor identity/generation |
| Input | `wheel_modules/operation_command` | Enable and reset transaction |
| Input | `vehicle/mode` | Supervisor authority and selected mode |
| Output | `can_tx` | SDO requests and master heartbeat |
| Output | `joint_states` | Complete actual position/velocity; empty effort |
| Output | `wheel_modules/status` | Startup, per-drive status, errors and current |
| Output | `diagnostics` | Leases, required field ages, scan span and sender health |

The supervisor owns public enable/reset. Internal initialize is virtual-only;
physical initialization belongs to supervisor startup. Brake, independent STO
and unavailable temperature remain unknown. DS402 status-word fields remain
unknown for interface compatibility; miControl native status is reported in
`device_state` with its own known flag and acquisition stamp.

## Validation

Tests cover native SDO widths and subindices, actual/requested enable separation,
malformed and replayed replies, complete feedback, initialization/readback,
velocity/current transitions, watchdogs, overspeed, alignment and sender pacing.
The virtual model is a protocol/kinematic fixture, not a motor or safety model.
Software socket-attempt spacing does not establish physical CAN wire spacing.
Hardware timing, units and stop behavior still require commissioning.

## Steering alignment on B2 enable

The active physical profile restores `steering_align_on_enable: true`. B2
clears/initializes/enables the controllers and aligns steering to its nearest
full-turn home at 1.2 rad/s, with traction held at zero. This uses the existing
encoder zero; it does not search for a home sensor or redefine the reference.
The configured manual policy also returns steering home on fresh neutral input,
after new complete feedback confirms traction standstill. Normal joystick
steering remains 0.3 rad/s; fresh stick motion cancels the neutral return.

`steering_align_on_enable` opts a physical profile into bounded nearest-home
alignment during each supervisor-authorized B2 recovery. Each steering target is
selected once from fresh post-enable feedback as the nearest full-turn equivalent
of the existing controller zero, using the configured position resolution. For
the installed 121 user units/turn, position 119 targets 121 instead of unwinding to 0; negative
and multiple turns work the same way. Exactly half-turn ties favor the original
zero. Targets stay fixed throughout the attempt. Invalid scaling or an
unrepresentable nearest target aborts recovery instead of choosing a longer move.
This never writes an encoder origin, searches for a home sensor, or saves
controller settings. Verify the configured position scale and physical home.

Traction remains zero and `startup_pending` stays true until all steering actual
positions are within one degree of their selected home (or a finer one-count tolerance) and fresh
steering velocities confirm standstill. The supervisor then completes its normal sustained
standstill proof before accepting fresh joystick motion. Controls must be neutral
to request recovery; subsequent stick movement is ignored until recovery ends.
Return to neutral afterward for fresh rearm. Defaults are `steering_alignment_velocity: 0.6` rad/s and
`steering_alignment_timeout: 12.5` s, allowing a half-turn at 0.6 rad/s. The
velocity is bounded by each motor native RPM limit and the timeout must fit within
`startup_timeout`. Ordinary operating-mode
changes do not request this B2 alignment phase.

Fresh management/authority and a matching reset transaction authorize the phase.
A safety stop, lost lease/authority, stale drive feedback, traction movement, or
an alignment timeout aborts it and disables all drives; a new B2 recovery is
required. The driver logs alignment start/completion and exposes the pending
phase in wheel-module status.


Traction `JointState.velocity` and final `traction_velocity` use contact m/s;
steering velocities remain rad/s and all feedback positions remain radians.
`wheel_radius` is derived from the platform for command conversion, measured
traction velocity, continuity bounds and current-mode overspeed checks. Steering
profile zero is applied with the documented miControl Update command, including
before enable; the virtual protocol models profile latching rather than stopping
an active move on a profile write alone.

Physical steering updates queue the positive speed profile and position target
as one acknowledged transaction. The target waits for its matching profile ACK;
new command samples replace the unsent target in place. An unchanged proven
profile is reused, while a changed profile requires the position to be latched
again even when the target angle is unchanged. A started pair finishes before
ordinary read scheduling can repeatedly postpone its target. Required fault
service, stop/disable priority, command expiry and the shared 2 ms send spacing
remain in force. Stops and lost native-state evidence revoke unsent moves;
a late acknowledgement cannot restore a revoked target.

Homing speed is independent of normal manual steering speed. The default
steering alignment profile is 0.6 rad/s; the physical SDO overlay retains
1.2 rad/s for optional alignment, which is currently disabled. Each steering motor's configured native RPM
limit bounds homing. The physical profile retains 0.3 rad/s for joystick
steering. Homing timeout, fresh-feedback proof and traction inhibition remain
in force.

The active four-miControl steering profile uses 121 native position units per
output revolution for 0x3762/0x3790, not 4096 raw encoder increments. A 90-degree
move rounds to 30 controller units (about 89.26 degrees). One native unit is about
2.98 degrees; the one-degree correction threshold therefore requires the reported
home count to match exactly. Normal manual alignment compares with the reachable
native target and resumes corrections if the wheel drifts by even one count;
deviation alone does not disable an enabled steering drive. This cannot prove
physical one-degree accuracy using coarse user-unit telemetry. Traction retains its
separate 4096-count motor encoder scale. The 4096-count home example above applies
only to controllers configured with that position-unit scale.
