#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class Converter(Node):
    def __init__(self):
        super().__init__('js_to_acker_converter')


        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheelbase', rclpy.Parameter.Type.DOUBLE),
                ('wheel_radius', rclpy.Parameter.Type.DOUBLE),
                ('wheels_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('wheels_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('traction_joint_names', rclpy.Parameter.Type.STRING_ARRAY),
                ('steering_joint_names', rclpy.Parameter.Type.STRING_ARRAY)]
        )

        self._wheelbase = self.get_parameter('wheelbase').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        wheels_x = self.get_parameter('wheels_x').value
        wheels_y = self.get_parameter('wheels_y').value
        traction_joint_names = self.get_parameter('traction_joint_names').value
        steering_joint_names = self.get_parameter('steering_joint_names').value

        self._wheels_info = []
        for x, y, traction_joint, steering_joint in zip(wheels_x, wheels_y, traction_joint_names, steering_joint_names):
            self._wheels_info.append({'x': x, 'y': y, 'traction_joint_name': traction_joint,
                                      'steering_joint_name': steering_joint})

        h = []
        h.extend([1, 0, -info['y'], 0, 1, info['x']] for info in self._wheels_info)
        self._Hjs = np.array(h, dtype=float).reshape(-1, 3)
        self._Hjs_inv = np.linalg.pinv(self._Hjs)

        self._Hacker = np.array([
            [1, 0, 0],
            [0, 0, self._wheelbase]
        ], dtype=float)
        self._Hacker_inv = np.linalg.pinv(self._Hacker)

        self.sub_joint_state = self.create_subscription(JointState, 'joint_states', self.js_callback, 10)
        self.pub_joint_state = self.create_publisher(JointState, 'joint_setpoints', 10)
        self.sub_ackermann = self.create_subscription(AckermannDriveStamped, 'cmd_acker', self.cmd_acker_callback, 10)
        self.pub_ackermann = self.create_publisher(AckermannDriveStamped, 'agv_acker', 10)

    def js_callback(self, msg_js):
        msg_acker = AckermannDriveStamped()
        msg_acker.header = msg_js.header
        msg_acker.drive.speed = 0.0
        msg_acker.drive.steering_angle = 0.0

        prep_list = []
        for info in self._wheels_info:
            try:
                idx_traction = msg_js.name.index(info['traction_joint_name'])
                idx_steering = msg_js.name.index(info['steering_joint_name'])
                vel = msg_js.velocity[idx_traction]
                steer = msg_js.position[idx_steering]
                prep_list.extend([vel * self._wheel_radius * np.cos(steer), vel * self._wheel_radius * np.sin(steer)])
            except ValueError:
                self.get_logger().warn(f"Joint for wheel at ({info['x']}, {info['y']}) not in JointState message. Skipping.")
                return

        v_acker = self._Hacker @ self._Hjs_inv @ np.array(prep_list, dtype=float).reshape(-1, 1)

        msg_acker.drive.speed = float(np.sqrt(v_acker[0] ** 2 + v_acker[1] ** 2))
        msg_acker.drive.steering_angle = float(np.arctan2(v_acker[1], v_acker[0]))

        if np.allclose(v_acker[:2], 0.0, atol=4e-3):
            positions = np.array([[info['x'], info['y']] for info in self._wheels_info])
            angles = np.array([msg_js.position[msg_js.name.index(info['steering_joint_name'])] for info in self._wheels_info])
            normals = np.stack([-np.cos(angles), -np.sin(angles)], axis=1)

            if np.allclose(np.abs(normals @ normals[0]), 1.0, atol=4e-3):
                msg_acker.drive.steering_angle = float(msg_js.position[msg_js.name.index(self._wheels_info[0]['steering_joint_name'])])
            else:
                icc, _, _, _ = np.linalg.lstsq(normals, np.sum(normals * positions, axis=1), rcond=None)
                direction = icc - np.array([0.0, self._wheelbase])
                msg_acker.drive.steering_angle = float(np.arctan2(direction[1], direction[0]) - np.pi / 2)

        if np.abs(msg_acker.drive.steering_angle) > np.pi / 2:
            msg_acker.drive.steering_angle += -np.sign(msg_acker.drive.steering_angle) * np.pi
            msg_acker.drive.speed *= -1

        self.pub_ackermann.publish(msg_acker)

    def cmd_acker_callback(self, msg_acker):
        msg_js = JointState()
        msg_js.header = msg_acker.header

        is_stopped = np.isclose(msg_acker.drive.speed, 0.0, atol=4e-3)
        speed = 1.0 if is_stopped else msg_acker.drive.speed
        v_js = self._Hjs @ self._Hacker_inv @ np.array([
            speed * np.cos(msg_acker.drive.steering_angle),
            speed * np.sin(msg_acker.drive.steering_angle)
        ], dtype=float)

        n = len(self._wheels_info)
        out_vel = np.zeros(n) if is_stopped else np.linalg.norm(v_js.reshape(n, 2), axis=1) / self._wheel_radius
        out_pos = np.arctan2(v_js[1::2], v_js[0::2])

        is_reverse = np.abs(out_pos) > np.pi / 2
        out_pos[is_reverse] += -np.sign(out_pos[is_reverse]) * np.pi
        out_vel[is_reverse] *= -1

        for i, info in enumerate(self._wheels_info):
            msg_js.name.append(info['traction_joint_name'])
            msg_js.velocity.append(float(out_vel[i]))
            msg_js.name.append(info['steering_joint_name'])
            msg_js.position.append(float(out_pos[i]))

        # self.pub_joint_state.publish(msg_js)

def main(args=None):
    rclpy.init(args=args)
    converter_node = Converter()
    rclpy.spin(converter_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()