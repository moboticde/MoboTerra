"""Validate ROS integration without CAN devices or motion commands."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from mobotic_interfaces.msg import WheelModuleStatusArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def main():
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
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('installation_smoke_check')
        samples = {}
        handles = [node.create_subscription(kind, topic,
                   lambda msg, key=key: samples.update({key: msg}), 10)
                   for kind, topic, key in (
                       (JointState, '/moboterra/joint_states', 'joints'),
                       (Odometry, '/moboterra/odometry', 'odom'),
                       (WheelModuleStatusArray, '/moboterra/wheel_modules/status', 'modules'),
                       (DiagnosticArray, '/moboterra/diagnostics', 'diagnostics'))]
        try:
            deadline = time.monotonic() + 60
            ready_since = None
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError('Virtual launch exited before validation')
                rclpy.spin_once(node, timeout_sec=0.2)
                names = {name for name, namespace in node.get_node_names_and_namespaces()
                         if namespace == '/moboterra'}
                ready = expected <= names and len(samples) == 4
                if ready:
                    if len(samples['joints'].name) != 8:
                        raise RuntimeError('Expected feedback for all eight joints')
                    if not samples['diagnostics'].status:
                        raise RuntimeError('Empty diagnostic publication')
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
            print('PASS: virtual nodes, eight joints, wheel status, odometry and diagnostics',
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
