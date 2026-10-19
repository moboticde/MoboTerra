"""Physical control assignments for the documented REMdevice factory PDOs."""
import copy
import re


class RemdeviceMapping:
    def __init__(self, mapping):
        if not isinstance(mapping, dict) or set(mapping) != {'verified', 'node_id', 'axes', 'buttons'}:
            raise ValueError('REMdevice mapping requires verified, node_id, axes, buttons')
        if type(mapping['verified']) is not bool:
            raise ValueError('REMdevice verified must be a boolean')
        node = mapping['node_id']
        if type(node) is not int or not 1 <= node <= 127:
            raise ValueError('REMdevice node_id must be 1..127')
        self.node_id = node
        self.verified = mapping['verified']
        self.axes = copy.deepcopy(mapping['axes'])
        self.buttons = copy.deepcopy(mapping['buttons'])
        if not isinstance(self.axes, dict) or not isinstance(self.buttons, dict):
            raise ValueError('REMdevice axes/buttons must be mappings')
        if not self.verified:
            if self.axes or self.buttons:
                raise ValueError('clear unverified assignments; use capture before mapping controls')
            return
        if set(self.axes) != {'speed', 'steer', 'crab'}:
            raise ValueError('REMdevice speed, steer and crab axes required')
        channels = set()
        for axis in self.axes.values():
            if not isinstance(axis, dict) or set(axis) != {'channel', 'zero', 'full_scale', 'invert'}:
                raise ValueError('axis requires channel, zero, full_scale, invert')
            if type(axis['channel']) is not int or not 1 <= axis['channel'] <= 8 or axis['channel'] in channels:
                raise ValueError('axis channels must be distinct A1..A8')
            channels.add(axis['channel'])
            if (type(axis['zero']) is not int or type(axis['full_scale']) is not int
                    or not 0 <= axis['zero'] < axis['full_scale'] <= 255
                    or type(axis['invert']) is not bool):
                raise ValueError('invalid REMdevice axis calibration/polarity')
        if set(self.buttons) != {'mode', 'boost', 'reset', 'deadman'}:
            raise ValueError('mode, boost, reset and explicit deadman (null to disable) required')
        signals = set()
        for role, button in self.buttons.items():
            if role == 'deadman' and button is None:
                continue
            if (not isinstance(button, dict) or set(button) != {'signal', 'active_high'}
                    or type(button['active_high']) is not bool
                    or not isinstance(button['signal'], str)
                    or not re.fullmatch(r'(SP|SA)([1-9]|1[0-6])', button['signal'])
                    or button['signal'] in signals):
                raise ValueError('buttons require distinct SP1..16/SA1..16 signals and boolean polarity')
            signals.add(button['signal'])

    def button(self, role, digital):
        spec = self.buttons[role]
        return False if spec is None else digital[spec['signal']] == spec['active_high']
