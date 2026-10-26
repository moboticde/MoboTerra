# mobotic_interfaces

Shared ROS 2 messages and services for the MoboTerra operator, vehicle supervisor, wheel modules, safety and battery interfaces. Rebuild this package and every dependent after message changes; mixed old/new schemas are unsupported.

## Interface rules

- `OperatorState` describes physical inputs, connection, source identity and neutral/rearm state. E-stop and deadman have explicit known flags.
- `VehicleModeState` describes effective and selected modes, error/safety overlap, readiness, reset progress and supervisor authority.
- `WheelModuleCommandArray` is final motion: one named record per configured module (two in the two-module profile, four by default), steering position/profile speed, traction velocity and provenance. No current command is exposed. See the [field reference](../../docs/control_path_and_latency.md#final-wheel-command-message).
- `WheelModuleOperationCommand` is internal management: enable and one idempotent reset ID, bound to supervisor instance/generation. It has no motion targets or clear-errors boolean.
- `WheelModuleStatusArray` contains one named record per configured module with separate `DriveStatus` for steering and traction. Currents are amperes; unknown values have known flags and NaN where appropriate.
- Native device-state/control-mode values are controller-family-specific. Status/position readiness must not be interpreted as physical E-stop, brake or STO.
- `ScannerSafetyState` exposes generic field/OSSD state and unknown field-set metadata. `SafetyTelegram` preserves gateway receipt time, connection epoch and sequence.
- `ComponentHealthArray` carries typed required-component health internally; it is not derived from the public mode's diagnostic severity.
- `joint_states` uses standard JointState for actual joint position [rad] and velocity [rad/s]. Effort is empty: amperes are not torque.
- Battery SOC/percentages are fractions (0.2 means 20%). Missing data does not mean zero or healthy.

Legacy mode aliases and ManualControlState remain generated for source compatibility only; there is no legacy runtime topic bridge. Brake/STO unknown flags are not permission to invent physical state.

See the [supervisor contract](../../docs/supervisor_interfaces.md) and [implementation plan](../../docs/implementation_plan.md). Hardware safety remains independent.

This package declares Apache-2.0. Retain package license/copyright notices and replace the placeholder maintainer address before release.
