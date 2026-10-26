"""Validate ROS integration without CAN devices or motion commands."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from mobotic_interfaces.msg import WheelModuleStatusArray, VehicleModeState, OperatorState, WheelModuleCommandArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from mobotic_config.configuration import Configuration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--platform-config', help='Optional platform profile to validate in virtual mode')
    args = parser.parse_args()
    cfg = Configuration(args.platform_config)
    expected_joints = set(cfg.parameters('driver')['drives.names'])
    expected_modules = set(cfg.parameters('supervisor')['expected_module_names'])
    # Keep build-time discovery local even when the Docker build has networking.
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '42')
    expected = {'mobotic_driver', 'mobotic_kinematics', 'mobotic_odometry',
                'mobotic_supervisor', 'platform_diagnostics'}
    with tempfile.TemporaryFile(mode='w+') as log:
        launch = subprocess.Popen([
            'ros2', 'launch', 'mobotic_bringup', 'moboterra.launch.py',
            'stack_type:=virtual', 'robot_name:=moboterra',
            'start_manual_interface:=false',
        ] + ([f'platform_config:={cfg.path}'] if args.platform_config else []),
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('installation_smoke_check')
        samples = {}
        received = {}
        def receive(key, msg):
            samples[key] = msg
            received[key] = time.monotonic()
        handles = [node.create_subscription(kind, topic,
                   lambda msg, key=key: receive(key, msg), 10)
                   for kind, topic, key in (
                       (JointState, '/moboterra/joint_states', 'joints'),
                       (Odometry, '/moboterra/vehicle/odom', 'odom'),
                       (WheelModuleStatusArray, '/moboterra/wheel_modules/status', 'modules'),
                       (DiagnosticArray, '/moboterra/diagnostics', 'diagnostics'),
                       (VehicleModeState, '/moboterra/vehicle/mode', 'mode'),
                       (WheelModuleCommandArray, '/moboterra/wheel_modules/command', 'commands'))]
        # This check launches only the virtual stack and no physical operator
        # receiver. Supply a clearly identified neutral virtual operator so the
        # supervisor must actually initialize/enable, without commanding motion.
        operator = node.create_publisher(OperatorState, '/moboterra/operator/state', 1)
        def publish_neutral_operator():
            msg = OperatorState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.source_id = 'installation_smoke_virtual_operator'
            msg.connected = msg.estop_state_known = msg.controls_neutral = True
            msg.command_active = msg.estop_pressed = False
            msg.status_message = 'virtual smoke check: neutral, no motion source'
            operator.publish(msg)
        operator_timer = node.create_timer(0.05, publish_neutral_operator)
        try:
            deadline = time.monotonic() + 60
            ready_since = None
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError('Virtual launch exited before validation')
                rclpy.spin_once(node, timeout_sec=0.2)
                names = {name for name, namespace in node.get_node_names_and_namespaces()
                         if namespace == '/moboterra'}
                ready = (expected <= names and {'joints', 'odom', 'modules', 'diagnostics', 'mode'} <= samples.keys()
                         and all(time.monotonic() - received[key] <= 0.5
                                 for key in ('joints', 'odom', 'modules', 'mode')))
                if ready:
                    if (len(samples['joints'].name) != len(expected_joints)
                            or set(samples['joints'].name) != expected_joints):
                        raise RuntimeError('Expected feedback for every configured joint')
                    if (len(samples['modules'].modules) != len(expected_modules)
                            or {m.name for m in samples['modules'].modules} != expected_modules):
                        raise RuntimeError('Expected status for every configured module')
                    if not samples['diagnostics'].status:
                        raise RuntimeError('Empty diagnostic publication')
                    mode = samples['mode']
                    ready = (not mode.error_active and not mode.safety_stop_active and not mode.reset_in_progress
                             and mode.readiness_known and mode.drives_enabled and mode.ready_to_drive
                             and all(m.startup_complete and m.initialized and m.enabled and m.position_valid
                                     and m.feedback_fresh and m.error_code == 0 for m in samples['modules'].modules))
                    if 'commands' in samples and any(m.traction_velocity != 0 or m.steering_profile_velocity != 0
                                                     for m in samples['commands'].modules):
                        raise RuntimeError('Unexpected motion command during neutral virtual readiness check')
                    if not ready:
                        ready_since = None
                        continue
                    ready_since = ready_since or time.monotonic()
                    if time.monotonic() - ready_since >= 3:
                        break
                else:
                    ready_since = None
            else:
                raise RuntimeError(f'Virtual startup timed out: nodes={sorted(names)}, '
                                   f'messages={sorted(samples)}')
            healthcheck = Path('/usr/local/bin/moboterra-healthcheck')
            if healthcheck.exists():
                subprocess.run([str(healthcheck)], check=True, timeout=15)
            print(f'PASS: virtual nodes, {len(expected_joints)} joints, '
                  f'{len(expected_modules)} enabled wheel modules, supervisor readiness, odometry and diagnostics',
                  flush=True)
        except Exception:
            log.seek(0)
            print(log.read()[-16000:], flush=True)
            raise
        finally:
            node.destroy_node()
            rclpy.shutdown()
            try:
                os.killpg(launch.pid, signal.SIGINT)
                launch.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait(timeout=5)
            except ProcessLookupError:
                pass


if __name__ == '__main__':
    main()
