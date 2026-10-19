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

Transitions stop and verify fresh body/all-joint standstill for 0.25 s before changing sources. A 10 s timeout latches ERROR. Safety recovery requires a fresh command; manual control also requires neutral/deadman rearm. Actual faults require `wheel_modules/clear_errors` (Trigger), sustained measured standstill and fresh driver acknowledgements. Reset acceptance is not proof of recovery.

Default command expiry is 250 ms, the control cycle 20 ms, startup grace 10 s and reset timeout 5 s. Policies, module names and limits come from `mobotic_config`. Monitoring loss, actual drive/BMS faults and required component health are root fault inputs; protective stops and low SOC are not themselves system errors.

```bash
ros2 topic echo /moboterra/operator/state
ros2 topic echo /moboterra/vehicle/mode
ros2 service call /moboterra/vehicle/set_mode mobotic_interfaces/srv/SetVehicleMode "{requested_mode: 2}"
ros2 service call /moboterra/wheel_modules/clear_errors std_srvs/srv/Trigger "{}"
```

Mode/reset calls affect the robot; use a secured platform. ROS supervision is not safety-rated. The physical safety chain remains independent, and this update is not hardware-commissioned.
