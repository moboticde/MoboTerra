"""Original ROMB binary plus production adapter on a local ROS topic bus; no CAN."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from mobotic_config.configuration import Configuration
from mobotic_kinematics.launching import kinematics_nodes
from mobotic_kinematics.romb.runtime import runtime_command


def test_original_solver_and_adapter_roundtrip():
    config = Configuration().parameters('kinematics')
    namespace = '/romb_solver_test'
    definitions = kinematics_nodes(lambda **kw: kw, namespace, config)
    processes = []
    logs = []
    rclpy.init()
    node = rclpy.create_node('romb_solver_test_bus')
    outputs, measured = [], []
    command = node.create_publisher(TwistStamped, namespace + '/vehicle/cmd_vel', 1)
    feedback = node.create_publisher(JointState, namespace + '/joint_states', 1)
    subscriptions = [
        node.create_subscription(JointState, namespace + '/kinematics/joint_setpoints', outputs.append, 10),
        node.create_subscription(TwistStamped, namespace + '/vehicle/measured_velocity', measured.append, 10)]
    steering, traction = config['steering_joint_names'], config['traction_joint_names']
    last_feedback = 0.
    forward_feedback = False

    def pump(predicate, seconds=8., publish_feedback=True):
        nonlocal last_feedback
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), 'ROMB/adapter exited unexpectedly'
            if publish_feedback and time.monotonic() - last_feedback > .05:
                msg = JointState(); msg.header.stamp = node.get_clock().now().to_msg()
                msg.name = steering + traction
                msg.position = ([-mount for mount in config['wheel_mount_angles']] + [0.] * len(traction)
                                if forward_feedback else [0.] * len(msg.name))
                msg.velocity = [0.] * len(steering) + ([.12] * len(traction) if forward_feedback else [0.] * len(traction))
                feedback.publish(msg); last_feedback = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.005)
            if predicate(): return True
        return False

    try:
        import tempfile
        for definition in definitions:
            arguments = ['--ros-args', '-r', '__ns:=' + namespace, '-r', '__node:=' + definition['name']]
            for source, destination in definition.get('remappings', []):
                arguments += ['-r', source + ':=' + destination]
            for key, value in definition['parameters'][0].items():
                arguments += ['-p', key + ':=' + json.dumps(value)]
            if definition['executable'] == 'romb_kinematics':
                executable, environment = runtime_command(arguments)
            else:
                executable = [sys.executable, '-m', 'mobotic_kinematics.romb.adapter', *arguments]
                environment = dict(os.environ)
            log = tempfile.TemporaryFile(mode='w+'); logs.append(log)
            processes.append(subprocess.Popen(executable, env=environment, stdout=log, stderr=log))
        assert pump(lambda: command.get_subscription_count() > 0 and len(measured) > 2, 15.)
        assert all(abs(value) < 1e-8 for value in
                   (measured[-1].twist.linear.x, measured[-1].twist.linear.y, measured[-1].twist.angular.z))
        for direction in (1., -1.):
            before = len(outputs)
            request = TwistStamped(); request.header.stamp = node.get_clock().now().to_msg()
            request.header.frame_id = 'base_link'; request.twist.linear.y = direction * .2
            command.publish(request)
            assert pump(lambda: len(outputs) > before), 'original ROMB produced no correlated wheel candidate'
            result = outputs[-1]
            assert result.header.stamp == request.header.stamp
            indices = {name:i for i,name in enumerate(result.name)}
            for steer, drive, mount in zip(steering, traction, config['wheel_mount_angles']):
                angle, speed = result.position[indices[steer]], result.velocity[indices[drive]]
                assert abs(speed * math.cos(angle + mount)) < 1e-4
                assert abs(speed * math.sin(angle + mount) - direction * .2) < 1e-4
        forward_feedback = True
        assert pump(lambda: abs(measured[-1].twist.linear.x - .12) < 1e-5), 'original direct kinematics did not report measured forward speed'
        assert abs(measured[-1].twist.linear.y) < 1e-5 and abs(measured[-1].twist.angular.z) < 1e-5
        pump(lambda: False, .4, publish_feedback=False)
        before = len(outputs)
        request.header.stamp = node.get_clock().now().to_msg(); command.publish(request)
        pump(lambda: False, .15, publish_feedback=False)
        assert len(outputs) == before, 'stale feedback must not authorize a new command'
    finally:
        for process in processes: process.terminate()
        for process in processes:
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
        for log in logs:
            log.seek(0); print(log.read()); log.close()
        node.destroy_node(); rclpy.shutdown()
