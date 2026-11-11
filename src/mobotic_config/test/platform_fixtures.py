"""Synthetic four-module operator fixture; never used as a hardware profile."""
import copy
from mobotic_config.configuration import Configuration, config_directory


def four_module_platform():
    profile = copy.deepcopy(Configuration(config_directory() / 'platform-four-micontrol.yaml').platform)
    profile['can']['master_node_id'] = '0x0B'
    profile['operator'].update(require_enable_interlock=False, require_neutral_rearm=False,
        receiver_can_overrun_advisory=False, receiver_auto_communication_reset=False)
    profile['operator']['mapping'] = {
        'verified': True, 'node_id': 10,
        'axes': {name: {'channel': channel, 'zero': 0, 'full_scale': 255, 'invert': False}
                 for name, channel in [('speed', 3), ('steer', 2), ('crab', 4)]},
        'buttons': {'mode': {'signal': 'U5', 'active_high': False},
                    'boost': {'signal': 'SP3', 'active_high': True},
                    'reset': {'signal': 'SP4', 'active_high': True},
                    'scanner_override': {'signal': 'D5', 'active_high': True}}}
    return profile
