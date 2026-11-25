"""Exercise virtual health/enable ordering without ROS or physical transports."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest.mock import Mock, patch


SOURCE = Path(__file__).resolve().parents[1] / 'checks/virtual_smoke.py'


def load_smoke():
    modules = {}
    for name, symbols in {
        'rclpy': (),
        'diagnostic_msgs.msg': ('DiagnosticArray',),
        'mobotic_interfaces.msg': ('WheelModuleStatusArray', 'VehicleModeState',
                                  'OperatorState', 'WheelModuleCommandArray'),
        'nav_msgs.msg': ('Odometry',),
        'sensor_msgs.msg': ('JointState',),
        'std_srvs.srv': ('Trigger',),
    }.items():
        module = types.ModuleType(name)
        for symbol in symbols:
            setattr(module, symbol, Mock())
        modules[name] = module
    spec = importlib.util.spec_from_file_location('virtual_smoke_under_test', SOURCE)
    smoke = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(smoke)
    return smoke


class VirtualSmokeTest(unittest.TestCase):
    def exercise(self, *, health_failure=False, unexpected_motion=False):
        smoke = load_smoke()
        events = []
        subscriptions = {}
        state = dict(enabled=False, clock=0.0)
        node = Mock()
        names = ('mobotic_driver', 'mobotic_kinematics', 'mobotic_odometry',
                 'mobotic_supervisor', 'platform_diagnostics')
        node.get_node_names_and_namespaces.return_value = [(name, '/moboterra') for name in names]
        node.create_subscription.side_effect = lambda kind, topic, callback, qos: subscriptions.update(
            {topic: callback})
        client = node.create_client.return_value
        client.service_is_ready.return_value = True

        def enable(_request):
            self.assertEqual(events, ['health'])
            events.append('enable')
            state['enabled'] = True
            state['enabled_at'] = state['clock']
            future = Mock()
            future.result.return_value = types.SimpleNamespace(success=True)
            return future

        client.call_async.side_effect = enable

        def spin(_node, **kwargs):
            mode = types.SimpleNamespace(
                readiness_known=True, drives_enabled=state['enabled'],
                ready_to_drive=state['enabled'], error_active=False,
                safety_stop_active=False, reset_in_progress=False,
                reason_codes=[] if state['enabled'] else ['B2_ENABLE_REQUIRED'])
            module = types.SimpleNamespace(
                name='wheel', startup_complete=True, initialized=True,
                enabled=state['enabled'], position_valid=True,
                feedback_fresh=True, error_code=0)
            command = types.SimpleNamespace(traction_velocity=1 if unexpected_motion else 0,
                                            steering_profile_velocity=0)
            messages = {
                'joint_states': types.SimpleNamespace(name=['steering', 'traction']),
                'vehicle/odom': object(),
                'wheel_modules/status': types.SimpleNamespace(modules=[module]),
                'diagnostics': types.SimpleNamespace(status=[object()]),
                'vehicle/mode': mode,
                'wheel_modules/command': types.SimpleNamespace(modules=[command]),
            }
            for topic, message in messages.items():
                subscriptions['/moboterra/' + topic](message)

        def health(_path, _node):
            self.assertFalse(state['enabled'])
            events.append('health')
            if health_failure:
                raise subprocess.CalledProcessError(1, 'healthcheck')

        def monotonic():
            state['clock'] += 0.01
            return state['clock']

        configuration = Mock()
        configuration.parameters.side_effect = lambda kind: {
            'driver': {'drives.names': ['steering', 'traction']},
            'supervisor': {'expected_module_names': ['wheel']},
        }[kind]
        launch = Mock(pid=123)
        launch.poll.return_value = None
        smoke.rclpy.init = Mock()
        smoke.rclpy.create_node = Mock(return_value=node)
        smoke.rclpy.spin_once = spin
        smoke.rclpy.try_shutdown = Mock()
        with patch.object(sys, 'argv', ['virtual_smoke.py']), \
                patch.object(smoke, 'Configuration', return_value=configuration), \
                patch.object(smoke, 'configure_discovery'), \
                patch.object(smoke, 'check_runtime_health', side_effect=health), \
                patch.object(smoke.Path, 'exists', return_value=True), \
                patch.object(smoke.subprocess, 'Popen', return_value=launch), \
                patch.object(smoke.os, 'killpg'), \
                patch.object(smoke.time, 'monotonic', side_effect=monotonic):
            smoke.main()
        return events, state

    def test_graph_health_precedes_enable_and_sustained_readiness(self):
        events, state = self.exercise()
        self.assertEqual(events, ['health', 'enable'])
        self.assertGreaterEqual(state['clock'] - state['enabled_at'], 3.0)

    def test_failed_graph_health_prevents_enable(self):
        with self.assertRaises(subprocess.CalledProcessError):
            self.exercise(health_failure=True)

    def test_neutral_readiness_still_rejects_motion_commands(self):
        with self.assertRaisesRegex(RuntimeError, 'Unexpected motion command'):
            self.exercise(unexpected_motion=True)


if __name__ == '__main__':
    unittest.main()
