#!/usr/bin/env python3

import rclpy
import time
import math
import threading
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped

class DataCollector:
    def __init__(self, node, topic_name):
        self.node = node
        self._lock = threading.Lock()
        self._messages = []
        self._is_collecting = False
        self._subscriber = self.node.create_subscription(TwistStamped, topic_name, self._callback, 10)
        self.node.get_logger().info("DataCollector subscribed to '{}'".format(topic_name))
    def _callback(self, msg):
        if self._is_collecting:
            with self._lock: self._messages.append(msg)
    def start_collection(self):
        with self._lock: self._messages, self._is_collecting = [], True
        time.sleep(0.1)
    def stop_and_get_average(self):
        self._is_collecting = False
        with self._lock: messages_copy = list(self._messages)
        if not messages_copy:
            self.node.get_logger().warn("No data collected during the stable period!"); return 0.0, 0.0, 0.0
        avg_vx = np.mean([m.twist.linear.x for m in messages_copy])
        avg_vy = np.mean([m.twist.linear.y for m in messages_copy])
        avg_wz = np.mean([m.twist.angular.z for m in messages_copy])
        return avg_vx, avg_vy, avg_wz

class RobotCommander:
    """
    Commands the robot using Twist messages for simple calibration motions.
    """
    def __init__(self, node, topic_name):
        self.node = node
        self._pub = self.node.create_publisher(TwistStamped, topic_name, 10)
        self.node.get_logger().info("RobotCommander publishing to '{}'".format(topic_name))
    def stop(self):
        self._publish_twist(0.0, 0.0)
    def execute_ramped_motion(self, data_collector, linear_x, angular_z, const_vel_duration, ramp_time):
        """ Executes a full ramp-up, constant-velocity, ramp-down motion. """
        start_time = self.node.get_clock().now()
        while ((self.node.get_clock().now() - start_time).nanoseconds / 1e9) < ramp_time and rclpy.ok():
            ratio = ((self.node.get_clock().now() - start_time).nanoseconds / 1e9) / ramp_time
            self._publish_twist(linear_x * ratio, angular_z * ratio)
            rclpy.spin_once(self.node, timeout_sec=0.02) # Replaces rospy.Rate(50).sleep()
        data_collector.start_collection()
        const_start_time = self.node.get_clock().now()
        while ((self.node.get_clock().now() - const_start_time).nanoseconds / 1e9) < const_vel_duration and rclpy.ok():
            self._publish_twist(linear_x, angular_z)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        start_ramp_down_time = self.node.get_clock().now()
        while ((self.node.get_clock().now() - start_ramp_down_time).nanoseconds / 1e9) < ramp_time and rclpy.ok():
            ratio = 1.0 - ((((self.node.get_clock().now() - start_ramp_down_time).nanoseconds / 1e9)) / ramp_time)
            self._publish_twist(linear_x * ratio, angular_z * ratio)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.stop()
        time.sleep(0.2)
        return data_collector.stop_and_get_average()
    def _publish_twist(self, linear_x, angular_z):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.twist.linear.x = float(linear_x)
        twist_msg.twist.angular.z = float(angular_z)
        self._pub.publish(twist_msg)

class LidarCalibrator:
    def __init__(self, node):
        self.node = node
        self._load_params()
        self.yaw_error_rad = 0.0
        self.x_offset_error = 0.0
        self.y_offset_error = 0.0
        rclpy.get_default_context().on_shutdown(self._shutdown_hook)
        self.data_collector = DataCollector(self.node, self.twist_topic)
        self.commander = RobotCommander(self.node, self.cmd_topic)
        time.sleep(1.0)
    def _load_params(self):
        self.node.declare_parameter('twist_topic', 'twist_estimate')
        self.twist_topic = self.node.get_parameter('twist_topic').value
        self.node.declare_parameter('cmd_vel', 'cmd_vel')
        self.cmd_topic = self.node.get_parameter('cmd_vel').value
        self.node.declare_parameter('drive_dist', 3.0)
        self.drive_dist = self.node.get_parameter('drive_dist').value
        self.node.declare_parameter('drive_speed', 0.3)
        self.drive_speed = self.node.get_parameter('drive_speed').value
        self.node.declare_parameter('turn_count', 2)
        self.turn_count = self.node.get_parameter('turn_count').value
        self.node.declare_parameter('turn_speed', 0.4)
        self.turn_speed = self.node.get_parameter('turn_speed').value
        self.node.declare_parameter('iterations', 3)
        self.iterations = self.node.get_parameter('iterations').value
        self.node.declare_parameter('accel_time', 1.5)
        self.accel_time = self.node.get_parameter('accel_time').value
    def _shutdown_hook(self):
        self.node.get_logger().info("Shutdown requested. Stopping the robot.")
        self.commander.stop()
    def run(self):
        self._print_welcome()
        try: input("Press Enter to begin calibration...")
        except NameError: raw_input("Press Enter to begin calibration...")
        if not rclpy.ok(): return
        self.calibrate_yaw()
        if not rclpy.ok(): return
        self.calibrate_position()
        if not rclpy.ok(): return
        self.generate_final_report()
    def _print_welcome(self):
        print("\n" + "="*60)
        print(" Lidar Extrinsic Calibration Tool")
        print("="*60)
        print("This script uses odometry-commanded motion to find the mounting")
        print("error of the lidar sensor group (yaw, x, y).")
        print("\n[IMPORTANT] Assumes odometry has been calibrated first!")

    def calibrate_yaw(self):
        print("\n--- PHASE 1: Calibrating Lidar Group YAW Offset ---")
        print("The robot will drive forward and back {} times.".format(self.iterations))

        yaw_errors = []
        for i in range(self.iterations):
            if not rclpy.ok(): break
            print("\nIteration {}/{}...".format(i + 1, self.iterations))
            const_vel_duration = self.drive_dist / self.drive_speed

            print("  Driving forward...")
            avg_vx, avg_vy, _ = self.commander.execute_ramped_motion(
                self.data_collector, self.drive_speed, 0.0, const_vel_duration, self.accel_time)

            iter_yaw_error = math.atan2(avg_vy, avg_vx)
            yaw_errors.append(iter_yaw_error)
            print("  Measured velocity vector: (vx={:.3f}, vy={:.3f}) -> Yaw Error: {:.2f} deg".format(
                avg_vx, avg_vy, math.degrees(iter_yaw_error)))

            print("  Returning to start...")
            avg_vx, avg_vy, _ = self.commander.execute_ramped_motion(
                self.data_collector, -self.drive_speed, 0.0, const_vel_duration, self.accel_time)

            iter_yaw_error = math.atan2(-avg_vy, -avg_vx)
            yaw_errors.append(iter_yaw_error)
            print("  Measured velocity vector: (vx={:.3f}, vy={:.3f}) -> Yaw Error: {:.2f} deg".format(
                avg_vx, avg_vy, math.degrees(iter_yaw_error)))

            time.sleep(1.0)

        if not yaw_errors:
            self.node.get_logger().fatal("Could not collect any valid yaw measurements. Aborting.")
            rclpy.shutdown()
            return

        self.yaw_error_rad = np.mean(yaw_errors)
        print("\n--- Phase 1 Complete ---")
        print("Final Yaw Offset Error: {:.3f} rad ({:.2f} deg)".format(
            self.yaw_error_rad, math.degrees(self.yaw_error_rad)))

    def calibrate_position(self):
        print("\n--- PHASE 2: Calibrating Lidar Group Positional (X, Y) Offset ---")
        try: input("Ensure robot has space to rotate {} times. Press Enter...".format(self.turn_count))
        except NameError: raw_input("Ensure robot has space to rotate {} times. Press Enter...".format(self.turn_count))
        if not rclpy.ok(): return

        const_vel_duration = abs(self.turn_count * 2 * math.pi / self.turn_speed)
        print("  Rotating for approx. {:.1f} seconds...".format(const_vel_duration))

        avg_vx, avg_vy, avg_wz = self.commander.execute_ramped_motion(
            self.data_collector, 0.0, self.turn_speed, const_vel_duration, self.accel_time)

        if abs(avg_wz) < 0.05:
            self.node.get_logger().fatal("Robot did not appear to rotate! Aborting position calibration.")
            rclpy.shutdown()
            return

        self.y_offset_error = -avg_vx / avg_wz
        self.x_offset_error = avg_vy / avg_wz

        print("\n--- Phase 2 Complete ---")
        print("  Measured induced linear velocity: (vx={:.4f}, vy={:.4f})".format(avg_vx, avg_vy))
        print("  Measured angular velocity: wz={:.4f}".format(avg_wz))
        print("Final X Offset Error: {:.4f} m".format(self.x_offset_error))
        print("Final Y Offset Error: {:.4f} m".format(self.y_offset_error))

    def generate_final_report(self):
        print("\n" + "="*71)
        print("||" + " " * 26 + "LIDAR CALIBRATION COMPLETE" + " " * 27 + "||")
        print("="*71 + "\n")

        print("[1] FINAL CALCULATED POSE ERRORS:")
        print("------------------------------------")
        print("This is the calculated offset of the lidar group from the center of rotation.")
        print("Yaw Error:      {:.4f} radians ({:.2f} degrees)".format(self.yaw_error_rad, math.degrees(self.yaw_error_rad)))
        print("X-Offset Error: {:.4f} meters".format(self.x_offset_error))
        print("Y-Offset Error: {:.4f} meters".format(self.y_offset_error))

        print("\n[2] ENTER CURRENT POSES TO GET CORRECTED VALUES:")
        print("-------------------------------------------------")
        print("(Press Enter with no input at the 'X offset' prompt to finish)")

        lidar_index = 1
        while rclpy.ok():
            print("\n--- Enter current pose for Lidar #{} ---".format(lidar_index))
            try:
                try: input_func = raw_input
                except NameError: input_func = input

                current_x_str = input_func("Enter CURRENT X offset (meters): ")
                if not current_x_str.strip(): break

                current_x = float(current_x_str)
                current_y = float(input_func("Enter CURRENT Y offset (meters): "))
                current_yaw_deg = float(input_func("Enter CURRENT Yaw (DEGREES): "))

            except (ValueError, EOFError):
                print("\nERROR: Invalid input. Please enter numeric values only. Try again.")
                continue
            except KeyboardInterrupt:
                break

            current_yaw_rad = math.radians(current_yaw_deg)

            x_rotated =  current_x * math.cos(self.yaw_error_rad) + current_y * math.sin(self.yaw_error_rad)
            y_rotated = -current_x * math.sin(self.yaw_error_rad) + current_y * math.cos(self.yaw_error_rad)

            new_x = x_rotated + self.x_offset_error
            new_y = y_rotated + self.y_offset_error

            new_yaw_rad = current_yaw_rad - self.yaw_error_rad

            print("\n--- CORRECTED Pose for Lidar #{} ---".format(lidar_index))
            print("New X:   {:.3f} meters".format(new_x))
            print("New Y:   {:.3f} meters".format(new_y))
            print("New Yaw: {:.3f} degrees".format(math.degrees(new_yaw_rad)))

            lidar_index += 1

        print("\n" + "="*71)
        print("Calibration finished.")
        print("="*71 + "\n")

def main():
    rclpy.init()
    node = rclpy.create_node('lidar_calibrator')
    try:
        calibrator = LidarCalibrator(node)
        calibrator.run()
    except KeyboardInterrupt:
        node.get_logger().info("Calibration interrupted.")
    except Exception as e:
        node.get_logger().fatal("An unhandled exception occurred: {}".format(e))
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
