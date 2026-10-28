#!/usr/bin/env python3
# _*_ coding: UTF-8 _*_

"""
The script controls individual joint actuators of a multi-wheel robot
according to data in sensor_msgs/JointState published on the joint_setpoints topic.

It is written in a generic way, to be applicable to an arbitrary number of wheels,
but has so far been tested onty with a 4-wheel vehicle (four wheels independently
actuated in steering and traction, 8 joints in total). Joint type is identified
by a 'steering' or 'traction' suffix in the joint name.
"""

from math import pi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi] interval."""
    return (angle + pi) % (2 * pi) - pi


class JointController(Node):
    def __init__(self):
        super().__init__("simulation_driver")

        self.setpoints = JointState()

        # Parameters
        self.declare_parameter('cmd_publish_rate', 1000)
        self.declare_parameter('wheel_radius', 0.1125)
        self.cmd_publish_rate = self.get_parameter('cmd_publish_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Publishers
        self._pubs_commands = dict()

        # Subscriptions
        # QoS (Quality of Service) for publishers and subscribers
        # The ROS 2 equivalent of queue_size=1 is a QoS profile with history depth 1
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.setpoint_sub = self.create_subscription(
            JointState,
            'joint_setpoints_pf',
            self.setpoint_callback,
            qos_profile
        )

        self.sub_states = self.create_subscription(
            JointState,
            'joint_states',
            self.cb_states,
            qos_profile
        )

        self.pub_states_pf = self.create_publisher(
            JointState,
            f'joint_states_pf',
            qos_profile
        )

        # Timer for periodic callbacks
        self.create_timer(1.0 / self.cmd_publish_rate, self.timer_callback)


    def cb_states(self, msg: JointState):
        feedback_msg = JointState()
        feedback_msg.header = msg.header
        feedback_msg.name = msg.name

        # Linear velocity v = omega * r
        if msg.velocity:
            feedback_msg.velocity = [float(omega * self.wheel_radius) for omega in msg.velocity]

        # Position (distance traveled) d = theta * r
        if msg.position:
            feedback_msg.position = [float(theta) for theta in msg.position]

        self.pub_states_pf.publish(feedback_msg)

    def setpoint_callback(self, data: JointState):
        self.setpoints = data
        if not self._pubs_commands:
            qos_profile = rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1
            )
            for joint_name in data.name:
                if joint_name.endswith('steering') or joint_name.endswith('traction'):
                    self._pubs_commands[joint_name] = self.create_publisher(
                        Float64MultiArray,
                        f'{joint_name}_controller/commands',
                        qos_profile
                    )
                else:
                    self.get_logger().warn(
                        f'Unknown joint type for joint {joint_name}'
                    )

    def timer_callback(self):
        if not self._pubs_commands:
            return

        cmd_type = None
        for joint_name, pub in self._pubs_commands.items():
            if not joint_name in self.setpoints.name:
                self.get_logger().warn(f"Joint {joint_name} not found in setpoints.")
                return

            multiplication_factor = 1.0
            if joint_name.endswith('steering'):
                cmd_type = self.setpoints.position
            elif joint_name.endswith('traction'):
                cmd_type = self.setpoints.velocity
                multiplication_factor = self.wheel_radius

            if len(self.setpoints.name) != len(cmd_type):
                #self.get_logger().warn(f"Length of setpoints name {len(self.setpoints.name)} doesn't match length of cmd_type: {len(cmd_type)}.")
                return



            if cmd_type is not None:
                msg = Float64MultiArray()
                #print(f"setpoints: {self.setpoints}")
                #print(f"cmd_type: {cmd_type}")
                #print(f"setpoints.name: {self.setpoints.name}")
                #print(f"setpoints.name.index: {self.setpoints.name.index(joint_name)}")
                #print(self.setpoints.name.index(joint_name))
                msg.data = [cmd_type[self.setpoints.name.index(joint_name)] / multiplication_factor]
                #print("SUCCESS!")
                pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ctrl = JointController()
    rclpy.spin(ctrl)
    ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
