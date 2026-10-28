#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class WorldTransformManager(Node):
    def __init__(self):
        super().__init__('world_map_transform_manager')

        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('initial_transfom_check_period', 1.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.static_broadcaster = StaticTransformBroadcaster(self)


        self._world_frame_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        self._map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self._base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        self._transform_check_period = self.get_parameter('initial_transfom_check_period').get_parameter_value().double_value

        self.srv = self.create_service(
            Trigger,
            'reset_world_position',
            self.reset_callback
        )

        self.initial_transform_set = False
        self.timer = self.create_timer(self._transform_check_period, self.initial_timer_callback)

    def initial_timer_callback(self):
        if not self.initial_transform_set:
            success, _ = self.update_and_publish_transform()
            if success:
                self.initial_transform_set = True
                self.timer.cancel()

    def update_and_publish_transform(self):


        try:
            t = self.tf_buffer.lookup_transform(
                self._base_frame_id,
                self._map_frame_id,
                rclpy.time.Time()
            )

            static_msg = TransformStamped()
            static_msg.header.stamp = self.get_clock().now().to_msg()
            static_msg.header.frame_id = self._world_frame_id
            static_msg.child_frame_id = self._map_frame_id

            static_msg.transform.translation = t.transform.translation
            static_msg.transform.rotation = t.transform.rotation

            self.static_broadcaster.sendTransform(static_msg)

            return True, "Successfully updated world -> map transform."

        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            return False, f"TF lookup failed: {ex}"

    def reset_callback(self, request, response):
        success, msg = self.update_and_publish_transform()

        response.success = success
        response.message = msg

        return response

def main(args=None):
    rclpy.init(args=args)
    node = WorldTransformManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
