# mobotic_driver

C++ ROS 2 driver for MoboTerra's four powered steering/traction modules. Steering uses MiControl; traction uses Elmo CANopen/DS402. Hardware configuration, scale/reference, gearing, limits and safety/brake behavior require commissioning.

The update configures and monitors drives while disabled, performs acknowledged PDO writes with readback, and accepts one final supervised motion array. Physical enable and authoritative joint publication remain inhibited pending verified Elmo position continuity; the synthetic position flag is only for virtual launch.

## Topics and services

All names are relative to the robot namespace, normally /moboterra.

| Direction | Topic | Type and role |
| --- | --- | --- |
| Input | can_rx | can_msgs/Frame; acquisition-stamped transport |
| Input | wheel_modules/command | WheelModuleCommandArray; single final motion input |
| Input | wheel_modules/operation_command | Internal WheelModuleOperationCommand; enable/reset ID |
| Input | vehicle/mode | VehicleModeState; supervisor authority and selected/effective state |
| Output | can_tx | can_msgs/Frame; commands, configuration and SYNC |
| Output | joint_states | JointState; complete actual position/velocity, empty effort |
| Output | wheel_modules/status | WheelModuleStatusArray; separate drive current/health |
| Output | diagnostics | DiagnosticArray; leases, CAN freshness and raw hardware readbacks |

There are no ordinary driver subscriptions to kinematics/joint_setpoints or supervisor/joint_setpoints. Supervisor owns public wheel_modules/clear_errors (Trigger). The remaining initialization/disable services are internal/driver/initialize_all_drives and internal/driver/disable_all_drives; initialization rejects enabled drives. A diagnostic disable is not a persistent supervisor override.

## Feedback and protocol

Elmo position is read immediately at initialization through 6064:0, then through configured TPDO3. No direct encoder access, encoder/firmware changes or automatic reference zeroing are performed. Raw readback is exposed as position_0x6064_raw in diagnostics. EDS mapping support does not prove the installed reference survives a full power cycle.

A joint publication requires position and velocity for all eight joints within one SYNC window and the configured sample-skew budget (5 ms by default). Its stamp is the oldest required motion acquisition, not report time. Traction counters are unwrapped with jump rejection; this alone cannot recover unpowered movement after a drive coordinate reset. See [controller object mapping](../../docs/controller_object_mapping.md).

Elmo 6078 current is per-mille of rated current 6075, not milliamps. The driver reads the rating and converts to amperes; stale/zero/absent rating yields unknown current. Steering uses the existing MiControl current conversion and still requires installed-unit verification. Current belongs in drive status, never JointState.effort.

Status records expose native state/mode, status word where supported, errors, current and per-field known flags/stamps. Brake, independent STO and unavailable temperature remain unknown. Elmo 2086/60FD are read-only raw diagnostics until their installed bit mappings are verified.

## Independent guards

| Guard | Default |
| --- | --- |
| Motion command watchdog | 100 ms |
| Supervisor management/mode leases | 300 ms |
| Motion and status feedback expiry | 300 ms |
| Fault register/code expiry | 2.5 s |
| SYNC period | 20 ms |
| Diagnostic/rated-current polling | 1 s |
| Configuration response deadline | 2 s per transaction |

Every final array must match supervisor instance, generation and selected mode, have ordered valid timestamps and contain each module once. Normal motion requires permission and a source age no greater than 250 ms. Controlled-stop authority only reduces traction without reversing; steering remains held. Actual controller mode and all-drive freshness are checked independently. Limits are derived from the shared profile, not fixed inside final-command validation.

Initialization and enable retries never clear faults. A reset ID is consumed once after fresh measured per-joint standstill (0.1 rad/s), then acknowledged only after new cleared fault readback. Supervisor verifies the complete transaction before clearing its latch. CAN boot-up stops/disables all drives and triggers disabled reconfiguration; lost position continuity cannot be repaired by clearing a fault.

Validated CAN samples are ordered per drive/field with the original acquisition watermark shared across PDO and SDO decoding. Heartbeats, unrelated SDOs and currents do not refresh motion. Zero-stamped transport has a local-reception compatibility fallback, not reliable replay detection; use the timestamping SocketCAN bridge on hardware.

## Shared profile

| Module | Steering CAN ID | Traction CAN ID |
| --- | --- | --- |
| front_left | 0x01 | 0x02 |
| front_right | 0x03 | 0x04 |
| rear_left | 0x07 | 0x08 |
| rear_right | 0x05 | 0x06 |

Joint names are <module>_steering and <module>_traction. Radius is 0.325 m; configured traction scaling is 4096 counts/revolution and 16:1 gearing. Raw traction ceiling is 116053 increments/s, with acceleration/deceleration 32093 increments/s². ROS limits remain 11.12 rad/s traction and 4.32 rad/s steering. These reference settings must match actual Elmo units before operation.

Use integrated bringup and the shared platform/policy YAML. The synchronized profile requires both PDO paths and a positive sample-skew budget below the SYNC period. Eight moving joints and four modules must match supervisor/kinematics configuration.

## Persistence and validation status

PositionStateStore provides versioned identity/calibration metadata, checksum validation, exclusive writer locking, atomic rename and fsync. Standalone tests cover its storage behavior, but a verified Elmo-only recovery backend and asynchronous writer are not yet integrated. The mounted state directory is reserved for that work; its existence is not a persistence guarantee.

The standalone CAN protocol, feedback ordering, routing, steering limits and new setup/position/current/storage tests pass on the local development host. Full Jazzy node compilation, live ROS/SocketCAN coherence and secured hardware power-loss tests remain release gates. See [supervisor contract](../../docs/supervisor_interfaces.md) and [implementation plan](../../docs/implementation_plan.md).

ROS monitoring is not safety-rated. Hardware STO, watchdogs and brake behavior remain independent.
