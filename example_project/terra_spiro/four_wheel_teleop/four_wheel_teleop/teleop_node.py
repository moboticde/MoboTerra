#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Twist
import numpy as np
from std_srvs.srv import Trigger, SetBool


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        # Parameters for scaling joystick inputs to linear and angular velocities.
        # Linear and angular velocity scaling factors are applied when not in "turbo" mode
        self.declare_parameters(
            namespace='',
            parameters=[
                ('axis_forward', rclpy.Parameter.Type.INTEGER),
                ('axis_reverse', rclpy.Parameter.Type.INTEGER),
                ('axis_steer', rclpy.Parameter.Type.INTEGER),
                ('axis_crab', rclpy.Parameter.Type.INTEGER),
                ('button_turbo_1', rclpy.Parameter.Type.INTEGER),
                ('button_turbo_2', rclpy.Parameter.Type.INTEGER),
                ('max_velocity_linear', rclpy.Parameter.Type.DOUBLE),
                ('max_velocity_angular', rclpy.Parameter.Type.DOUBLE),
                ('max_acceleration_linear', rclpy.Parameter.Type.DOUBLE),
                ('max_acceleration_angular', rclpy.Parameter.Type.DOUBLE),
                ('scale_linear', rclpy.Parameter.Type.DOUBLE),
                ('scale_angular', rclpy.Parameter.Type.DOUBLE),
                ('button_enable', rclpy.Parameter.Type.INTEGER),
                ('button_disable', rclpy.Parameter.Type.INTEGER),
                ('button_clear', rclpy.Parameter.Type.INTEGER),
                ('commands_timeout_sec', rclpy.Parameter.Type.DOUBLE),
                ('commands_publish_sec', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        self._is_enabled = True

        self.ax_forward = self.get_parameter('axis_forward').value
        self.ax_reverse = self.get_parameter('axis_reverse').value
        self.ax_steer = self.get_parameter('axis_steer').value
        self.ax_crab = self.get_parameter('axis_crab').value
        self.turbo_1 = self.get_parameter('button_turbo_1').value
        self.turbo_2 = self.get_parameter('button_turbo_2').value
        self.enable = self.get_parameter('button_enable').value
        self.disable = self.get_parameter('button_disable').value
        self.clear = self.get_parameter('button_clear').value
        self.max_vel_lin = self.get_parameter('max_velocity_linear').value
        self.max_vel_ang = self.get_parameter('max_velocity_angular').value
        self.max_acc_lin = self.get_parameter(
            'max_acceleration_linear').value
        self.max_acc_ang = self.get_parameter(
            'max_acceleration_angular').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.cmd_timeout_threshold = self.get_parameter('commands_timeout_sec').value
        self.cmd_publish_time = self.get_parameter('commands_publish_sec').value

        self.twist = TwistStamped()

        self.cmd_vel_pub = self.create_publisher(TwistStamped,
                                                 'cmd_vel',  1)
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 1)

        self.client_init_drives = self.create_client(Trigger, 'initialize_all_drives')
        self.client_enable_drives = self.create_client(Trigger, 'enable_all_drives')
        self.client_disable_drives = self.create_client(Trigger, 'disable_all_drives')
        self.client_clear_errors = self.create_client(Trigger, 'clear_errors')

        self.enable_service_ = self.create_service(
            SetBool,
            'enable_teleop',
            self.handle_enable_disable
        )


        self.cmd_publish_timer = self.create_timer(self.cmd_publish_time, self.cmd_publish_callback, callback_group=None)

    def handle_enable_disable(self, request, response):
        if request.data:
            if self._is_enabled:
                response.success = True
                response.message = "Node is already enabled."
            else:
                self._is_enabled = True
                response.success = True
                response.message = "Node has been enabled successfully."
        else:
            if not self._is_enabled:
                response.success = True
                response.message = "Node is already disabled."
            else:
                self._is_enabled = False
                response.success = True
                response.message = "Node has been disabled successfully."

        self.get_logger().info(response.message)

        # 5. Return the populated response
        return response



    def joy_callback(self, joy_msg):
        """ Convert joystick axes to linear and angular velocities. """

        if joy_msg.buttons[self.enable]:
            self.client_init_drives.call_async(Trigger.Request())
            self.client_enable_drives.call_async(Trigger.Request())
            self.get_logger().info('Enabling drives...')
            return

        if joy_msg.buttons[self.disable]:
            self.client_disable_drives.call_async(Trigger.Request())
            self.get_logger().info('Disabling drives...')
            return

        if joy_msg.buttons[self.clear]:
            self.client_clear_errors.call_async(Trigger.Request())
            self.get_logger().info('Clearing errors...')
            return

        speed_pct = 0.5 * (joy_msg.axes[self.ax_reverse] -
                           joy_msg.axes[self.ax_forward])
        steer_pct = joy_msg.axes[self.ax_steer]
        crab_pct = joy_msg.axes[self.ax_crab]
        V = (np.array([self.max_vel_lin, self.max_vel_lin, self.max_vel_ang])
             * np.array([(1 - abs(crab_pct)) * (1 - abs(steer_pct)),
                         crab_pct * (1 - abs(steer_pct)),
                         (1 - abs(crab_pct)) * steer_pct])) * speed_pct

        # Apply joystick enabling and scaling
        turbo = joy_msg.buttons[self.turbo_1] or joy_msg.buttons[self.turbo_2]
        V *= np.array([turbo + (1-turbo) * self.scale_linear,
                       turbo + (1-turbo) * self.scale_linear,
                       turbo + (1-turbo) * self.scale_angular])

        # Apply acceleration limits
        dt = (Time.from_msg(joy_msg.header.stamp) -
              Time.from_msg(self.twist.header.stamp)).nanoseconds * 1.e-9
        V_previous = np.array([self.twist.twist.linear.x,
                               self.twist.twist.linear.y,
                               self.twist.twist.angular.z])
        A = np.array([self.max_acc_lin, self.max_acc_lin, self.max_acc_ang])
        np.clip(V, V_previous - A * dt, V_previous + A * dt, out=V)

        self.twist.header = joy_msg.header
        self.twist.twist.linear.x = V[0]
        self.twist.twist.linear.y = V[1]
        self.twist.twist.angular.z = V[2]

    def cmd_publish_callback(self):
        if self.get_clock().now() - Time.from_msg(self.twist.header.stamp) > Duration(
                seconds=self.cmd_timeout_threshold):
            self.twist.header.stamp = self.get_clock().now().to_msg()
            self.twist.twist = Twist()
        if self._is_enabled:
            self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        teleop.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
