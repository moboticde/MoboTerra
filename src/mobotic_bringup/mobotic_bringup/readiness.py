"""Read-only configuration and live readiness report; never sends drive commands."""
import argparse
import json
import math
import time


def configuration_report(cfg):
    platform = cfg.platform
    operator = platform.get('operator', {'source': 'usb'})
    blockers = []
    if operator['source'] == 'remdevice' and not operator['mapping']['verified']:
        blockers.append('PAIL control mapping is unverified; capture-only mode')
    if operator['source'] == 'usb' and not cfg.parameters('manual')['input_mapping_verified']:
        blockers.append('USB control mapping is unverified')
    return {
        'platform_config': str(cfg.path), 'configuration_ready': not blockers,
        'operator_source': operator['source'],
        'receiver_node': operator['mapping']['node_id'] if operator['source'] == 'remdevice' else None,
        'master_node': platform['can']['master_node_id'],
        'drive_can': platform['can']['drives'], 'battery_can': platform['can']['battery'],
        'modules': [module['name'] for module in platform['modules']],
        'safety_source': platform['safety'].get('source', 'flexisoft'),
        'blockers': blockers, 'live_checked': False, 'software_ready': None,
        'hardware_verified': False,
        'commissioning_pending': platform.get('commissioning_pending', []),
    }


def live_report(cfg, samples, now_ns):
    result = configuration_report(cfg)
    blockers = result['blockers']
    policy = cfg.parameters('supervisor')

    def fresh(key, timeout):
        message, received = samples.get(key, (None, None))
        if message is None or received is None or not 0 <= now_ns - received <= timeout * 1e9:
            blockers.append(key + ' missing or stale')
            return None
        stamp = message.header.stamp
        acquired = stamp.sec * 1_000_000_000 + stamp.nanosec
        if (stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or acquired <= 0
                or not -100_000_000 <= now_ns - acquired <= timeout * 1e9):
            blockers.append(key + ' acquisition timestamp invalid or stale')
            return None
        return message

    mode = fresh('vehicle/mode', cfg.parameters('driver')['supervisor_timeout'])
    if mode is not None:
        result['supervisor_reason'] = mode.status_message
        result['motion_permitted'] = mode.motion_permitted
        if mode.error_active or mode.reset_in_progress:
            blockers.extend(mode.reason_codes or ['supervisor error/reset active'])
        if mode.safety_stop_active:
            blockers.append('safety stop active')
        if not (mode.readiness_known and mode.drives_enabled and mode.ready_to_drive):
            blockers.append(mode.status_message or 'supervisor is not ready to drive')

    modules = fresh('wheel_modules/status', policy['wheel_status_timeout'])
    if modules is not None:
        names = [module.name for module in modules.modules]
        if len(names) != len(result['modules']) or set(names) != set(result['modules']):
            blockers.append('wheel-module status does not match selected profile')
        for module in modules.modules:
            if module.startup_failed:
                blockers.append(module.name + ': startup failed')
            elif not module.startup_complete:
                blockers.append(module.name + ': startup pending or not requested')
            elif not (module.initialized and module.enabled and module.position_valid and module.feedback_fresh) or module.error_code:
                blockers.append(module.name + ': ' + (module.status_message or 'drive feedback/enable/fault not ready'))

    battery = fresh('battery/system_state', policy['battery_timeout'])
    if battery is not None and (not battery.communication_ok or not battery.system_ready or
            not battery.high_voltage_connected or battery.faulted_batteries or
            not math.isfinite(battery.minimum_percentage) or
            not policy['minimum_battery_percentage'] <= battery.minimum_percentage <= 1):
        blockers.append('battery: ' + (battery.status_message or 'telemetry/HV/SOC/fault not ready'))

    operator_required = result['operator_source'] == 'remdevice' or mode is None or mode.selected_mode == 0
    operator = fresh('operator/state', policy['manual_timeout']) if operator_required else None
    if operator is not None:
        if not operator.connected:
            blockers.append('operator: ' + (operator.status_message or 'disconnected'))
        if operator.estop_pressed:
            blockers.append('operator E-stop active')
        if result['operator_source'] == 'remdevice' and not operator.estop_state_known:
            blockers.append('PAIL STOP/link state unknown')
    result.update(blockers=list(dict.fromkeys(blockers)), live_checked=True)
    result['software_ready'] = not result['blockers']
    return result


def main(args=None):
    from mobotic_config.configuration import Configuration, ConfigurationError, COMPONENT_FILES
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--platform-config')
    parser.add_argument('--namespace', help='Defaults to robot_name from the selected profile')
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--offline', action='store_true', help='Validate configuration without ROS or CAN access')
    parser.add_argument('--override', action='append', default=[], metavar='COMPONENT=PATH',
                        help='Match a policy overlay used by the running stack; may be repeated')
    options, ros_args = parser.parse_known_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    try:
        overrides = {}
        for item in options.override:
            component, separator, path = item.partition('=')
            if not separator or not path or component not in COMPONENT_FILES or component in overrides:
                raise ConfigurationError('--override requires a unique known COMPONENT=PATH')
            overrides[component] = path
        cfg = Configuration(options.platform_config, overrides)
    except ConfigurationError as error:
        parser.exit(2, str(error) + '\n')
    if options.offline:
        result = configuration_report(cfg)
    else:
        import rclpy
        from mobotic_interfaces.msg import VehicleModeState, WheelModuleStatusArray, BatterySystemState, OperatorState
        rclpy.init(args=ros_args)
        namespace = options.namespace if options.namespace is not None else cfg.platform['robot_name']
        node = rclpy.create_node('moboterra_readiness_check', namespace=namespace)
        samples = {}
        subscriptions = [node.create_subscription(kind, topic,
            lambda msg, key=topic: samples.update({key: (msg, node.get_clock().now().nanoseconds)}), 1)
            for kind, topic in ((VehicleModeState, 'vehicle/mode'), (WheelModuleStatusArray, 'wheel_modules/status'),
                                (BatterySystemState, 'battery/system_state'), (OperatorState, 'operator/state'))]
        try:
            deadline = time.monotonic() + options.timeout
            while True:
                rclpy.spin_once(node, timeout_sec=0.1)
                result = live_report(cfg, samples, node.get_clock().now().nanoseconds)
                if result['software_ready'] or time.monotonic() >= deadline:
                    break
        finally:
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps(result, indent=2))
    if not (result['software_ready'] if result['live_checked'] else result['configuration_ready']):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
