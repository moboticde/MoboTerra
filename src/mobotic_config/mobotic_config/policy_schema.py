"""Parameter types only; default values belong exclusively to config/*.yaml."""

POLICY_TYPES = {
    "battery": {
        "telemetry_timeout": "number",
        "publish_period": "number"
    },
    "driver": {
        "feedback_request_period": "number",
        "can_heartbeat_period": "number",
        "watchdog_timeout": "number",
        "supervisor_timeout": "number",
        "motion_feedback_timeout": "number",
        "status_feedback_timeout": "number",
        "fault_feedback_timeout": "number",
        "diagnostic_sdo_period": "number",
        "traction_pdo_enabled": "boolean",
        "telemetry_pdo_enabled": "boolean",
        "telemetry_pdo_sync_divider": "integer",
        "traction_pdo_period": "number"
    },
    "kinematics": {
        "cmd_vel_timeout": "number",
        "joint_feedback_timeout": "number",
        "minimum_linear_speed": "number"
    },
    "manual": {
        "axis_speed": "integer",
        "axis_crab": "integer",
        "axis_steer": "integer",
        "button_deadman": "integer",
        "buttons_boost": "integer_list",
        "button_mode_switch": "integer",
        "mode_switch_type": "string",
        "mode_switch_manual_value": "integer",
        "mode_switch_auto_velocity_value": "integer",
        "mode_state_timeout": "number",
        "scale_linear": "number",
        "scale_angular": "number",
        "joystick_timeout": "number",
        "publish_period": "number"
    },
    "odometry": {
        "publish_tf": "boolean",
        "feedback_timeout": "number",
        "max_integration_interval": "number",
        "initial_pose": "number_list",
        "initial_pose_variances": "number_list",
        "twist_variances": "number_list",
        "process_variance_rates": "number_list",
        "gap_variance_rates": "number_list",
        "unobserved_variance": "number",
        "max_linear_speed": "number",
        "max_angular_speed": "number"
    },
    "safety": {
        "accept_timeout": "number",
        "status_timeout": "number",
        "publish_period": "number",
        "estop_implies_sto": "boolean"
    },
    "scanners": {
        "interface_ip": "string",
        "host_udp_port": "integer",
        "channel": "integer",
        "channel_enabled": "boolean",
        "skip": "integer",
        "angle_start": "number",
        "angle_end": "number",
        "time_offset": "number",
        "general_system_state": "boolean",
        "derived_settings": "boolean",
        "measurement_data": "boolean",
        "intrusion_data": "boolean",
        "application_io_data": "boolean",
        "use_persistent_config": "boolean",
        "min_intensities": "number"
    },
    "supervisor": {
        "initial_mode": "string",
        "control_period": "number",
        "mode_transition_stop_duration": "number",
        "mode_transition_timeout": "number",
        "velocity_feedback_timeout": "number",
        "standstill_linear_velocity": "number",
        "standstill_angular_velocity": "number",
        "standstill_joint_velocity": "number",
        "manual_timeout": "number",
        "autonomy_timeout": "number",
        "safety_timeout": "number",
        "battery_timeout": "number",
        "wheel_status_timeout": "number",
        "minimum_battery_percentage": "number",
        "warning_speed_scale": "number",
        "override_speed_scale": "number"
    },
    "diagnostics": {
        "scanner_timeout": "number",
        "odometry_timeout": "number",
        "publish_period": "number"
    }
}
