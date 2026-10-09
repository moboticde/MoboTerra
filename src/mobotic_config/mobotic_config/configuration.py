"""Validated platform data -> unchanged node parameter APIs. No ROS imports at load time."""
import argparse
import copy
import ipaddress
import json
import math
import re
from pathlib import Path
import warnings

import yaml
from .policy_schema import POLICY_TYPES


COMPONENT_FILES = {
    'driver': 'driver.yaml', 'kinematics': 'kinematics.yaml',
    'manual': 'manual_control.yaml', 'supervisor': 'supervisor.yaml',
    'safety': 'safety.yaml', 'battery': 'battery.yaml', 'odometry': 'odometry.yaml',
    'scanners': 'scanners.yaml', 'diagnostics': 'diagnostics.yaml',
}
MODULE_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')


class ConfigurationError(ValueError):
    pass


class UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ConfigurationError(f'YAML mapping keys must be strings at line {key_node.start_mark.line + 1}')
        if key in result:
            raise ConfigurationError(f'duplicate YAML key {key!r} at line {key_node.start_mark.line + 1}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def read_yaml(path):
    try:
        value = yaml.load(Path(path).read_text(encoding='utf-8'), Loader=UniqueLoader)
    except (OSError, yaml.YAMLError) as error:
        raise ConfigurationError(f'{path}: {error}') from error
    if not isinstance(value, dict):
        raise ConfigurationError(f'{path}: expected a YAML mapping')
    return value


def config_directory():
    # Source-tree fallback is for offline validation only; installed ROS wins.
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mobotic_config')) / 'config'
    except ImportError:
        return Path(__file__).resolve().parents[1] / 'config'


def default_platform_path():
    return str(config_directory() / 'platform.yaml')


def config_path(component):
    return str(config_directory() / COMPONENT_FILES[component])


def _number(value, label, *, positive=False, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigurationError(f'{label}: expected a finite number')
    if integer and not isinstance(value, int):
        raise ConfigurationError(f'{label}: expected an integer')
    if positive and value <= 0:
        raise ConfigurationError(f'{label}: must be positive')
    return value


def _text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f'{label}: expected a non-empty string')
    return value


def _id(value, maximum, label):
    try:
        result = int(value, 0) if isinstance(value, str) else value
    except ValueError as error:
        raise ConfigurationError(f'{label}: invalid integer') from error
    _number(result, label, integer=True)
    if not 0 <= result <= maximum:
        raise ConfigurationError(f'{label}: outside 0..{maximum}')
    return result


def _vector(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise ConfigurationError(f'{label}: expected three coordinates')
    for number in value:
        _number(number, label)


def _ip(value, label):
    _text(value, label)
    try:
        ipaddress.IPv4Address(value)
    except (ValueError, TypeError, ipaddress.AddressValueError) as error:
        raise ConfigurationError(f'{label}: invalid IPv4 address') from error


def _record(value, keys, label, required=None):
    if not isinstance(value, dict):
        raise ConfigurationError(f'{label}: expected a mapping')
    unknown = set(value) - set(keys)
    missing = set(keys if required is None else required) - set(value)
    if unknown or missing:
        raise ConfigurationError(f'{label}: missing={missing}, unknown={unknown}')


def validate_platform(platform):
    try:
        top = ('schema_version', 'robot_name', 'frames', 'can', 'geometry', 'actuators',
               'traction_profile', 'motion_limits', 'modules', 'usb_joystick',
               'scanners', 'safety', 'battery')
        _record(platform, (*top, 'commissioning_pending'), 'platform', required=top)
        _record(platform['frames'], ('base', 'odom'), 'frames')
        _record(platform['geometry'], ('wheel_radius', 'wheel_width'), 'geometry')
        _record(platform['can'], ('master_node_id', 'drives', 'battery'), 'can')
        _record(platform['can']['drives'], ('interface', 'bitrate'), 'can.drives')
        _record(platform['can']['battery'], ('interface', 'bitrate', 'listen_only'), 'can.battery')
        _record(platform['actuators'], ('steering', 'traction'), 'actuators')
        actuator_keys = ('resolution', 'gear_ratio', 'mode', 'min_steering_velocity',
                         'max_steering_velocity', 'min_traction_velocity', 'max_traction_velocity',
                         'min_current', 'max_current')
        for role in ('steering', 'traction'):
            _record(platform['actuators'][role], actuator_keys, 'actuators.' + role)
        _record(platform['traction_profile'], ('max_velocity', 'acceleration', 'deceleration'), 'traction_profile')
        _record(platform['motion_limits'], ('max_velocity_linear', 'max_velocity_angular', 'max_acceleration_linear',
                'max_acceleration_angular', 'max_steering_velocity', 'max_traction_velocity', 'max_traction_current'), 'motion_limits')
        _record(platform['usb_joystick'], ('device_id', 'autorepeat_rate', 'deadzone', 'sticky_buttons'), 'usb_joystick')
        _record(platform['scanners'], ('host_ip', 'units'), 'scanners')
        _record(platform['battery'], ('primary_id', 'participants'), 'battery')
        if type(platform['schema_version']) is not int or platform['schema_version'] != 1:
            raise ConfigurationError('schema_version must be 1')
        _text(platform['robot_name'], 'robot_name')
        if not re.fullmatch(r'/?[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*', platform['robot_name']):
            raise ConfigurationError('robot_name must be a valid ROS namespace')
        for name in ('base', 'odom'):
            _text(platform['frames'][name], f'frames.{name}')
            if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', platform['frames'][name]):
                raise ConfigurationError('frame IDs must be simple URDF-safe names')
        if platform['frames']['base'] == platform['frames']['odom']:
            raise ConfigurationError('base and odom frames must differ')
        for name in ('wheel_radius', 'wheel_width'):
            _number(platform['geometry'][name], name, positive=True)
        for name in ('max_velocity_linear', 'max_velocity_angular', 'max_acceleration_linear',
                     'max_acceleration_angular', 'max_steering_velocity', 'max_traction_velocity',
                     'max_traction_current'):
            _number(platform['motion_limits'][name], name, positive=True)
        for name in ('max_velocity', 'acceleration', 'deceleration'):
            _number(platform['traction_profile'][name], name, positive=True, integer=True)
            if platform['traction_profile'][name] > 2**31 - 1:
                raise ConfigurationError(f'traction_profile.{name}: must fit the driver signed 32-bit parameter')
        can = platform['can']
        master = _id(can['master_node_id'], 127, 'master CAN ID')
        if master == 0:
            raise ConfigurationError('CAN node IDs cannot be zero')
        for bus in ('drives', 'battery'):
            _text(can[bus]['interface'], f'can.{bus}.interface')
            if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', can[bus]['interface']):
                raise ConfigurationError('CAN interface must be a valid Linux device name (1..15 characters)')
            _number(can[bus]['bitrate'], 'CAN bitrate', positive=True, integer=True)
        if can['drives']['interface'] == can['battery']['interface']:
            raise ConfigurationError('drive and battery CAN interfaces must be separate')
        if can['battery']['listen_only'] is not True:
            raise ConfigurationError('battery transport must remain listen-only')
        modules = platform['modules']
        if not isinstance(modules, list) or [m['name'] for m in modules] != list(MODULE_NAMES):
            raise ConfigurationError(f'modules must be ordered as {MODULE_NAMES}')
        ids, joints = [master], []
        for module in modules:
            _record(module, ('name', 'cad_steering_xyz', 'mounting_yaw', 'steering', 'traction'), 'module')
            _vector(module['cad_steering_xyz'], 'CAD steering datum')
            _number(module['mounting_yaw'], 'mounting yaw')
            for role in ('steering', 'traction'):
                _record(module[role], (*actuator_keys, 'joint', 'can_node_id'), module['name'] + '.' + role,
                        required=('joint', 'can_node_id'))
                drive = {**platform['actuators'][role], **module[role]}
                joint = _text(drive['joint'], 'joint name')
                if not joint.endswith('_' + role):
                    raise ConfigurationError(f'{joint}: driver requires _{role} suffix')
                if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', joint):
                    raise ConfigurationError(f'{joint}: invalid URDF joint name')
                joints.append(joint)
                node_id = _id(drive['can_node_id'], 127, joint + ' CAN ID')
                if node_id == 0:
                    raise ConfigurationError('CAN node IDs cannot be zero')
                ids.append(node_id)
                _number(drive['resolution'], 'encoder resolution', positive=True, integer=True)
                _number(drive['gear_ratio'], 'gear ratio', positive=True)
                if drive['mode'] != ('position' if role == 'steering' else 'velocity'):
                    raise ConfigurationError('this MoboTerra profile uses position steering / velocity traction')
                for quantity in ('steering_velocity', 'traction_velocity', 'current'):
                    lower = _number(drive['min_' + quantity], quantity, integer=True)
                    upper = _number(drive['max_' + quantity], quantity, integer=True)
                    if not lower < 0 < upper:
                        raise ConfigurationError(f'{joint}: invalid {quantity} bounds')
                    if not -(2**31) <= lower < upper <= 2**31 - 1:
                        raise ConfigurationError(f'{joint}: controller bounds must fit signed 32-bit integers')
                limits = platform['motion_limits']
                if role == 'traction':
                    ticks = limits['max_traction_velocity'] * drive['resolution'] * drive['gear_ratio'] / (2 * math.pi)
                    ceiling = min(drive['max_traction_velocity'], -drive['min_traction_velocity'],
                                  platform['traction_profile']['max_velocity'])
                    if ticks > ceiling or limits['max_traction_current'] * 1000 > min(drive['max_current'], -drive['min_current']):
                        raise ConfigurationError(f'{joint}: software traction limits exceed controller limits')
                else:
                    rpm = limits['max_steering_velocity'] * drive['gear_ratio'] * 60 / (2 * math.pi)
                    if rpm > min(drive['max_steering_velocity'], -drive['min_steering_velocity']):
                        raise ConfigurationError(f'{joint}: software steering limit exceeds controller limit')
        if len(set(ids)) != len(ids) or len(set(joints)) != len(joints):
            raise ConfigurationError('CAN IDs and joint names must be unique')
        if len({tuple(m['cad_steering_xyz'][:2]) for m in modules}) != 4:
            raise ConfigurationError('four wheel axes must have distinct XY coordinates')
        scanners = platform['scanners']
        _ip(scanners['host_ip'], 'scanner host')
        if [s['name'] for s in scanners['units']] != ['front_left', 'rear_right']:
            raise ConfigurationError('expected front_left and rear_right scanners')
        for scanner in scanners['units']:
            _record(scanner, ('name', 'sensor_ip', 'frame_id', 'cad_xyz', 'yaw'), 'scanner')
            _ip(scanner['sensor_ip'], 'scanner IP')
            _text(scanner['frame_id'], 'scanner frame')
            if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', scanner['frame_id']):
                raise ConfigurationError('scanner frames must be simple URDF-safe names')
            _vector(scanner['cad_xyz'], 'scanner CAD origin')
            _number(scanner['yaw'], 'scanner yaw')
        if len({s['frame_id'] for s in scanners['units']}) != 2 or len({s['sensor_ip'] for s in scanners['units']}) != 2:
            raise ConfigurationError('scanner frames and IPs must be unique')
        generated_links = {'chassis', platform['frames']['base']}
        for module in modules:
            generated_links.update(module['name'] + suffix for suffix in ('_steering_mount', '_steering_link', '_wheel'))
        if len(generated_links) != 14 or any(s['frame_id'] in generated_links for s in scanners['units']):
            raise ConfigurationError('base and scanner frames conflict with generated links')
        if platform['frames']['odom'] in generated_links or platform['frames']['odom'] in {s['frame_id'] for s in scanners['units']}:
            raise ConfigurationError('odom frame cannot belong to the robot description tree')
        safety = platform['safety']
        mask_names = ('front_protective_ossd_mask', 'rear_protective_ossd_mask',
                      'emergency_stop_mask', 'safety_override_mask', 'rear_warning_mask',
                      'front_warning_mask', 'safety_enable_mask', 'sto_mask')
        _record(safety, ('bind_address', 'port', 'telegram_size', 'status_byte_offset', *mask_names), 'safety')
        _ip(safety['bind_address'], 'FlexiSoft bind address')
        _number(safety['port'], 'TCP port', integer=True)
        if not 1 <= _id(safety['port'], 65535, 'TCP port'):
            raise ConfigurationError('TCP port must be nonzero')
        size = _number(safety['telegram_size'], 'telegram size', integer=True, positive=True)
        offset = _number(safety['status_byte_offset'], 'status offset', integer=True)
        if not 0 <= offset < size:
            raise ConfigurationError('status offset must be inside telegram')
        masks = [_id(_number(safety[key], key, integer=True), 255, key) for key in mask_names]
        occupied = 0
        for mask in masks:
            if occupied & mask:
                raise ConfigurationError('safety signal masks overlap')
            occupied |= mask
        battery = platform['battery']
        participants = battery['participants']
        if not isinstance(participants, list) or not participants:
            raise ConfigurationError('battery participants must be nonempty')
        identifiers, telegrams = [], []
        for participant in participants:
            _record(participant, ('id', 'source_address', 'status_pgn', 'soc_pgn', 'voltage_pgn'), 'battery participant')
            identifiers.append(_text(participant['id'], 'battery ID'))
            address = _id(participant['source_address'], 255, 'battery address')
            for field in ('status_pgn', 'soc_pgn', 'voltage_pgn'):
                telegrams.append((address, _id(participant[field], 0x3FFFF, field)))
        if len(set(identifiers)) != len(identifiers) or len(set(telegrams)) != len(telegrams):
            raise ConfigurationError('battery IDs/telegram bindings must be unique')
        if battery['primary_id'] not in identifiers:
            raise ConfigurationError('primary battery must be a participant')
        joystick = platform['usb_joystick']
        _id(joystick['device_id'], 65535, 'USB device index')
        if not 0 <= _number(joystick['deadzone'], 'deadzone') < 1:
            raise ConfigurationError('deadzone must be in [0, 1)')
        _number(joystick['autorepeat_rate'], 'autorepeat rate', positive=True)
        if joystick['sticky_buttons'] is not False:
            raise ConfigurationError('sticky buttons would change boost/mode semantics')
    except (KeyError, TypeError, AttributeError) as error:
        raise ConfigurationError(f'incomplete/invalid platform record: {error}') from error


def ros_parameters(path):
    document = read_yaml(path)
    if set(document) != {'/**'} or not isinstance(document['/**'], dict):
        raise ConfigurationError(f'{path}: overrides must use /**: ros__parameters:')
    parameters = document['/**'].get('ros__parameters')
    if not isinstance(parameters, dict):
        raise ConfigurationError(f'{path}: missing ros__parameters mapping')
    return parameters


class Configuration:
    def __init__(self, platform_path=None, overrides=None):
        self.path = Path(default_platform_path() if platform_path is None else platform_path).resolve()
        self.platform = read_yaml(self.path)
        validate_platform(self.platform)
        self._derived = self._derive()
        self._parameters = {}
        for component, filename in COMPONENT_FILES.items():
            defaults = ros_parameters(config_path(component))
            schema = POLICY_TYPES[component]
            if set(defaults) != set(schema):
                raise ConfigurationError(f'{component}: policy keys must match schema; missing={set(schema) - set(defaults)}, unknown={set(defaults) - set(schema)}')
            defaults = {key: self._policy_value(value, schema[key], f'{component}.{key}')
                        for key, value in defaults.items()}
            # Physical parameters are derived once; node files contain policy only.
            self._parameters[component] = {**defaults, **self._derived.get(component, {})}
        for component, path in (overrides or {}).items():
            if path:
                self.apply_overrides(component, ros_parameters(path))
        self._validate_policy()

    def _derive(self):
        p = self.platform
        modules = p['modules']
        drives = [(role, {**p['actuators'][role], **m[role]}) for m in modules for role in ('steering', 'traction')]
        driver = {'can_node_id': str(p['can']['master_node_id']),
                  'traction_max_profile_velocity': p['traction_profile']['max_velocity'],
                  'traction_profile_accel': p['traction_profile']['acceleration'],
                  'traction_profile_decel': p['traction_profile']['deceleration']}
        for key, field in (('names', 'joint'), ('can_node_ids', 'can_node_id'), ('modes', 'mode'),
                           ('resolutions', 'resolution'), ('gear_ratios', 'gear_ratio'),
                           ('min_steering_velocities', 'min_steering_velocity'),
                           ('max_steering_velocities', 'max_steering_velocity'),
                           ('min_traction_velocities', 'min_traction_velocity'),
                           ('max_traction_velocities', 'max_traction_velocity'),
                           ('min_currents', 'min_current'), ('max_currents', 'max_current')):
            driver['drives.' + key] = [float(d[field]) if field == 'gear_ratio' else
                                       str(d[field]) if field == 'can_node_id' else d[field] for _, d in drives]
        names = [m['name'] for m in modules]
        steering = [m['steering']['joint'] for m in modules]
        traction = [m['traction']['joint'] for m in modules]
        resolutions = [float({**p['actuators']['steering'], **m['steering']}['resolution']) for m in modules]
        driver.update({'modules.names': names, 'modules.steering_drives': steering, 'modules.traction_drives': traction})
        limits, frame = p['motion_limits'], p['frames']['base']
        vehicle_limits = {key: float(limits[key]) for key in ('max_velocity_linear', 'max_velocity_angular',
                                                            'max_acceleration_linear', 'max_acceleration_angular')}
        joint_limits = {key: float(limits[key]) for key in ('max_steering_velocity', 'max_traction_velocity', 'max_traction_current')}
        kinematics = {'module_names': names, 'steering_joint_names': steering, 'traction_joint_names': traction,
                      'wheels_x': [float(m['cad_steering_xyz'][0]) for m in modules],
                      'wheels_y': [float(m['cad_steering_xyz'][1]) for m in modules],
                      'wheel_mount_angles': [float(m['mounting_yaw']) for m in modules],
                      'wheel_radius': float(p['geometry']['wheel_radius']),
                      'traction_control_mode': 'velocity', 'output_frame_id': frame, **joint_limits}
        supervisor = {'expected_module_names': names, 'expected_steering_joint_names': steering,
                      'expected_traction_joint_names': traction, 'direct_steering_encoder_resolutions': resolutions,
                      'direct_traction_control_mode': 'velocity', 'output_frame_id': frame, **vehicle_limits}
        supervisor.update({'max_direct_' + key.removeprefix('max_'): value for key, value in joint_limits.items()})
        participants = p['battery']['participants']
        battery = {'primary_battery_id': p['battery']['primary_id']}
        for key, field in (('ids', 'id'), ('source_addresses', 'source_address'), ('status_pgns', 'status_pgn'),
                           ('soc_pgns', 'soc_pgn'), ('voltage_pgns', 'voltage_pgn')):
            battery['batteries.' + key] = [str(b[field]) for b in participants]
        return {'driver': driver, 'kinematics': kinematics, 'supervisor': supervisor,
                'manual': {'frame_id': frame, **vehicle_limits}, 'battery': battery,
                'safety': copy.deepcopy(p['safety']),
                'odometry': {'base_frame_id': frame, 'odom_frame_id': p['frames']['odom']}}

    def parameters(self, component):
        return copy.deepcopy(self._parameters[component])

    def apply_overrides(self, component, values, *, legacy=False):
        if component not in self._parameters:
            raise ConfigurationError(f'Unknown component: {component}')
        current = self._parameters[component]
        for key, value in values.items():
            if key not in current:
                raise ConfigurationError(f'{component}: unknown override {key}')
            if legacy:
                warnings.warn(f'{key} launch argument is deprecated; use platform_config / parameters_file',
                              UserWarning, stacklevel=2)
            expected = current[key]
            if key in self._derived.get(component, {}):
                # A lower manual source cap is allowed; platform enforcement stays authoritative.
                if component == 'manual' and key.startswith(('max_velocity_', 'max_acceleration_')):
                    _number(value, key, positive=True)
                    if value > self._derived[component][key]:
                        raise ConfigurationError(f'{component}.{key}: cannot exceed platform limit')
                elif not self._equivalent(key, value, self._derived[component][key]):
                    raise ConfigurationError(f'{component}.{key}: conflicts with platform_config {self.path}; edit the platform profile')
            if key == 'can_node_id' or key == 'drives.can_node_ids':
                # Accept equivalent decimal/hex legacy IDs but keep the canonical
                # string representation expected by the C++ driver.
                value = copy.deepcopy(self._derived[component][key])
            current[key] = self._coerce(value, expected, component + '.' + key)

    @staticmethod
    def _policy_value(value, kind, label):
        if kind.endswith('_list'):
            if not isinstance(value, list):
                raise ConfigurationError(f'{label}: expected list')
            return [Configuration._policy_value(v, kind.removesuffix('_list'), label) for v in value]
        if kind in ('number', 'integer'):
            result = _number(value, label, integer=kind == 'integer')
            return float(result) if kind == 'number' else result
        if kind == 'boolean' and type(value) is not bool:
            raise ConfigurationError(f'{label}: expected boolean')
        if kind == 'string' and not isinstance(value, str):
            raise ConfigurationError(f'{label}: expected string')
        return value

    @staticmethod
    def _equivalent(key, value, expected):
        if key == 'can_node_id':
            return _id(value, 127, key) == _id(expected, 127, key)
        if key == 'drives.can_node_ids':
            return isinstance(value, list) and [_id(v, 127, key) for v in value] == [_id(v, 127, key) for v in expected]
        return value == expected

    @staticmethod
    def _coerce(value, expected, label):
        if isinstance(expected, list):
            if not isinstance(value, list):
                raise ConfigurationError(f'{label}: expected list')
            return [Configuration._coerce(v, expected[0], label) for v in value] if expected else value
        if isinstance(expected, bool):
            if type(value) is not bool:
                raise ConfigurationError(f'{label}: expected boolean')
            return value
        if isinstance(expected, float):
            return float(_number(value, label))
        if isinstance(expected, int):
            return _number(value, label, integer=True)
        if isinstance(expected, str):
            if not isinstance(value, str):
                raise ConfigurationError(f'{label}: expected string')
            return value
        return value

    def _validate_policy(self):
        for component, parameters in self._parameters.items():
            for key, value in parameters.items():
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    _number(value, f'{component}.{key}')
                    if ('timeout' in key or key.endswith('_period')) and value <= 0:
                        raise ConfigurationError(f'{component}.{key}: must be positive')
                    if (key.startswith(('max_', 'standstill_')) or key in ('mode_transition_stop_duration', 'minimum_linear_speed')) and value <= 0:
                        raise ConfigurationError(f'{component}.{key}: must be positive')
                if isinstance(value, list):
                    for item in value:
                        if isinstance(item, (int, float)) and not isinstance(item, bool):
                            _number(item, f'{component}.{key}')
        percentage = self._parameters['supervisor']['minimum_battery_percentage']
        if not 0 <= percentage <= 1:
            raise ConfigurationError('minimum_battery_percentage must be a fraction in [0, 1]')
        manual = self._parameters['manual']
        if manual['mode_switch_type'] not in ('toggle', 'position'):
            raise ConfigurationError('mode_switch_type must be toggle or position')
        if {manual['mode_switch_manual_value'], manual['mode_switch_auto_velocity_value']} != {0, 1}:
            raise ConfigurationError('mode switch values must be distinct digital values 0 and 1')
        if manual['button_deadman'] < -1 or manual['button_mode_switch'] < -1:
            raise ConfigurationError('disabled button indices must be -1')
        boosts = manual['buttons_boost']
        if not boosts or len(set(boosts)) != len(boosts) or any(i < 0 for i in boosts):
            raise ConfigurationError('boost button indices must be nonempty, unique and nonnegative')
        if any(manual[k] < 0 for k in ('axis_speed', 'axis_crab', 'axis_steer')):
            raise ConfigurationError('axis indices must be nonnegative')
        if manual['button_deadman'] >= 0 and manual['button_deadman'] in boosts:
            raise ConfigurationError('deadman and boost cannot overlap')
        if manual['button_mode_switch'] >= 0 and manual['button_mode_switch'] in [manual['button_deadman'], *boosts]:
            raise ConfigurationError('mode switch and driving buttons cannot overlap')
        for key in ('scale_linear', 'scale_angular'):
            if not 0 <= manual[key] <= 1:
                raise ConfigurationError(f'{key} must be in [0, 1]')
        supervisor = self._parameters['supervisor']
        if supervisor['initial_mode'] not in ('manual', 'auto_velocity', 'auto_direct'):
            raise ConfigurationError('invalid initial_mode')
        for key in ('warning_speed_scale', 'override_speed_scale'):
            if not 0 <= supervisor[key] <= 1:
                raise ConfigurationError(f'{key} must be in [0, 1]')
        odometry = self._parameters['odometry']
        for key in ('initial_pose', 'initial_pose_variances', 'twist_variances', 'process_variance_rates', 'gap_variance_rates'):
            if len(odometry[key]) != 3 or (key != 'initial_pose' and any(v < 0 for v in odometry[key])):
                raise ConfigurationError(f'{key}: expected three finite values (variances nonnegative)')
        if self._parameters['driver']['telemetry_pdo_sync_divider'] not in range(1, 241):
            raise ConfigurationError('telemetry_pdo_sync_divider must be an integer in 1..240')

    def virtual_drives(self):
        result = []
        for module in self.platform['modules']:
            for role in ('steering', 'traction'):
                drive = {**self.platform['actuators'][role], **module[role]}
                result.append((drive['joint'], {'can_node_id': str(drive['can_node_id']), 'drive_type': role,
                                               'encoder_resolution': float(drive['resolution']),
                                               'gear_ratio': float(drive['gear_ratio'])}))
        return result

    def scanner_parameters(self, location):
        scanner = next(s for s in self.platform['scanners']['units'] if s['name'] == location)
        return {**self.parameters('scanners'), 'sensor_ip': scanner['sensor_ip'],
                'host_ip': self.platform['scanners']['host_ip'], 'frame_id': scanner['frame_id']}

    def diagnostic_parameters(self):
        supervisor, driver = self.parameters('supervisor'), self.parameters('driver')
        return {**self.parameters('diagnostics'), 'minimum_battery_percentage': supervisor['minimum_battery_percentage'],
                'manual_timeout': supervisor['manual_timeout'], 'battery_timeout': supervisor['battery_timeout'],
                'safety_timeout': supervisor['safety_timeout'], 'mode_timeout': driver['supervisor_timeout']}


def main(args=None):
    parser = argparse.ArgumentParser(description='Read-only MoboTerra configuration check; does not start ROS or hardware.')
    parser.add_argument('--platform-config', default=default_platform_path())
    parser.add_argument('--override', action='append', default=[], metavar='COMPONENT=PATH',
                        help='Optional policy overlay; repeat for different components')
    options = parser.parse_args(args)
    try:
        overrides = {}
        for item in options.override:
            component, separator, path = item.partition('=')
            if not separator or not path or component not in COMPONENT_FILES or component in overrides:
                raise ConfigurationError('--override requires a unique known COMPONENT=PATH')
            overrides[component] = path
        cfg = Configuration(options.platform_config, overrides)
    except ConfigurationError as error:
        parser.exit(1, f'Invalid configuration: {error}\n')
    print(json.dumps({'platform_config': str(cfg.path), 'parameters': cfg._parameters,
                      'virtual_drives': cfg.virtual_drives(), 'diagnostics': cfg.diagnostic_parameters(),
                      'commissioning_pending': cfg.platform.get('commissioning_pending', [])}, indent=2))


if __name__ == '__main__':
    main()
