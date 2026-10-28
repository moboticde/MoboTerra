#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rclpy
import math
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState

class JointStateConverter(Node):
    def __init__(self):
        super().__init__("joint_state_converter")

        # Parameters
        self.declare_parameter('wheel_radius', 0.1125)
        self.declare_parameter('max_steering_velocity', 2.16)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_velocity = self.get_parameter('max_steering_velocity').value

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- FEEDBACK: Angular -> Linear ---
        self.sub_states = self.create_subscription(
            JointState, 'joint_states', self.feedback_callback, qos_profile)

        self.pub_states_pf = self.create_publisher(
            JointState, 'agv_js', qos_profile)

        # --- SETPOINTS: Linear -> Angular ---
        self.sub_setpoints_pf = self.create_subscription(
            JointState, 'cmd_js', self.setpoint_callback, qos_profile)

        self.pub_setpoints_raw = self.create_publisher(
            JointState, 'joint_setpoints', qos_profile)

        self._agv_js = JointState()

        self.get_logger().info(f"Converter Node Online (Radius: {self.wheel_radius}, max_steering_velocity {self.max_steering_velocity})")

    def feedback_callback(self, msg: JointState):
        """Converts robot feedback to linear units (meters)."""
        self._agv_js = msg
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.name = msg.name

        new_vel = []
        for i, name in enumerate(msg.name):
            if 'traction' in name:
                sign = -1 if 'front' in name else 1
                new_vel.append(sign * float(msg.velocity[i] * self.wheel_radius))
            else:
                new_vel.append(float(msg.velocity[i]))

        out_msg.effort = msg.effort
        out_msg.position = msg.position
        out_msg.velocity = new_vel
        self.pub_states_pf.publish(out_msg)


    def setpoint_callback(self, msg: JointState):
        """Converts linear setpoints back to rotational units (radians)."""
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.name = msg.name

        new_vel = []
        for i, name in enumerate(msg.name):
            if 'traction' in name:
                sign = -1 if 'front' in name else 1
                new_vel.append(sign * float(msg.velocity[i] / self.wheel_radius))
            elif 'steering' in name:
                if name in self._agv_js.name:
                    feedback = self._agv_js.position[self._agv_js.name.index(name)]
                else:
                    feedback = 0.0
                setpoint = msg.position[msg.name.index(name)]

                new_vel.append(math.copysign(self.max_steering_velocity, setpoint - feedback))
            else:
                new_vel.append(msg.velocity[i])

        out_msg.position = msg.position
        out_msg.velocity = new_vel
        self.pub_setpoints_raw.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateConverter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
