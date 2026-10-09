# mobotic_interfaces

Shared ROS 2 messages for MoboTerra's driver, supervisor, manual control, safety
and battery monitoring, plus the `SetVehicleMode` service.

## Interface rules

- `WheelModuleCommand` contains a header, `enable` and `clear_errors`; it manages
  **all** configured units together. No joint setpoints or `brake_release` field.
- `WheelModuleStatusArray` identifies modules by `name`; order has no meaning.
  Its header is report time, not proof of simultaneous/fresh CAN acquisition.
- Each `WheelModuleStatus.feedback_fresh` explicitly describes required decoded
  feedback validity. A new array containing stale cached values cannot authorize motion.
- `brake_state_known=false` means brake release cannot be confirmed from feedback.
- Motion uses standard `sensor_msgs/msg/JointState` on dedicated kinematics/supervisor
  inputs; driver feedback uses `joint_states` with the oldest required motion sample timestamp.
- `SafetyState` is the high-level ROS motion-permission result.
  `SafetyIOState` exposes decoded inputs, raw FlexiSoft status and
  `sto_state_known` / `safety_enable_state_known`. Unknown signals must not be
  presented as verified hardware safety.
- Battery percentages, including aggregate `minimum_percentage`, are fractions:
  `0.2` means 20%. Missing/invalid telemetry blocks readiness.
- Missing/stale safety data is handled conservatively. Hardware STO remains independent.

See the [supervisor contract](../../docs/supervisor_interfaces.md).
Rebuild this package **and its dependents** after message changes.

This package declares Apache-2.0. Retain each package's own license/copyright
notices. The placeholder maintainer address must be replaced before release.
