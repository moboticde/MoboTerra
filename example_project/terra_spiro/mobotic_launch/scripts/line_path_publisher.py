#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Header

# Import your custom messages and services
from agv_msgs.msg import PathStamped
from roadmap_search_msgs.srv import PlanPathById
from follow_path_msgs.action import FollowPath
from follow_path_msgs.srv import AppendPath

class SimpleLinePathPublisher(Node):
    def __init__(self):
        super().__init__('simple_line_path_publisher')

        # Node name parameters
        self.declare_parameter('left_node_name', 'left_0')
        self.declare_parameter('center_node_name', 'center_0')
        self.declare_parameter('right_node_name', 'right_0')
        self.declare_parameter('map_frame_id', 'world')

        self._left_node = self.get_parameter('left_node_name').get_parameter_value().string_value
        self._center_node = self.get_parameter('center_node_name').get_parameter_value().string_value
        self._right_node = self.get_parameter('right_node_name').get_parameter_value().string_value
        self._map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value

        self.sub_active_segments_ = self.create_subscription(
            PathStamped,
            'active_segments',
            self.cb_active_segments,
            1
        )

        self._cl_plan_path = self.create_client(PlanPathById, 'get_shortest_path_by_id')
        self._cl_path_append = self.create_client(AppendPath, 'append_path')
        self._cl_path_follow = ActionClient(self, FollowPath, 'path_follower')

        while not self._cl_plan_path.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "get_shortest_path_by_id" service...')

        while not self._cl_path_append.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "append_path" service...')

        while not self._cl_path_follow.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for "follow_path" action server...')

        self._path_cache = dict()
        self._last_appended_target = None

        # Start the back-and-forth movement
        self.send_initial_goal()

    def send_path_as_goal(self, path):
        path_stamped = PathStamped()
        path_stamped.header.stamp = self.get_clock().now().to_msg()
        path_stamped.header.frame_id = self._map_frame_id
        path_stamped.path = path

        goal_msg = FollowPath.Goal()
        goal_msg.path_stamped = path_stamped

        self._cl_path_follow.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def send_initial_goal(self):
        # Initial trajectory: Start at center, head to the right
        begin_node = self._center_node
        end_node = self._right_node

        self.get_logger().info(f"Requesting initial plan from {begin_node} to {end_node}.")
        request = PlanPathById.Request()
        request.node_start = begin_node
        request.node_end = end_node

        self._last_appended_target = self._right_node
        future = self._cl_plan_path.call_async(request)
        future.add_done_callback(self.send_future_as_goal)

    def send_path_to_append(self, path):
        path_stamped = PathStamped()
        path_stamped.header.stamp = self.get_clock().now().to_msg()
        path_stamped.header.frame_id = self._map_frame_id
        path_stamped.path = path

        request = AppendPath.Request()
        request.path = path_stamped
        self._cl_path_append.call_async(request)

    def send_future_to_append(self, future):
        try:
            path = future.result().path
            self._path_cache[(path.segments[0].begin.id, path.segments[-1].end.id)] = path
            self.send_path_to_append(path)
        except Exception as e:
            self.get_logger().error(f'Append service call failed: {e}. Cannot send path to append.')

    def send_future_as_goal(self, future):
        try:
            path = future.result().path
            self._path_cache[(path.segments[0].begin.id, path.segments[-1].end.id)] = path
            self.send_path_as_goal(path)
        except Exception as e:
            self.get_logger().error(f'Planning service call failed: {e}. Cannot send goal path.')

    def feedback_callback(self, feedback_msg):
        self.get_logger().debug(f'Received feedback: {feedback_msg}')

    def append_path(self, begin_node, end_node):
        if (begin_node, end_node) in self._path_cache:
            self.send_path_to_append(self._path_cache[(begin_node, end_node)])
            return

        request = PlanPathById.Request()
        request.node_start = begin_node
        request.node_end = end_node

        future = self._cl_plan_path.call_async(request)
        future.add_done_callback(self.send_future_to_append)

    def cb_active_segments(self, msg):
        if not msg.path.segments:
            return

        curr_seg_begin = msg.path.segments[0].begin.id
        curr_seg_end = msg.path.segments[-1].end.id

        self.get_logger().info(f"Currently on segment from {curr_seg_begin} to {curr_seg_end}.")

        # If we are on a segment heading to the right node, queue up the return trip to the left
        if curr_seg_end == self._right_node and self._last_appended_target != self._left_node:
            self.get_logger().info(f"Approaching Right. Appending path back to Left.")
            self.append_path(self._right_node, self._left_node)
            self._last_appended_target = self._left_node

        # If we are on a segment heading to the left node, queue up the return trip to the right
        elif curr_seg_end == self._left_node and self._last_appended_target != self._right_node:
            self.get_logger().info(f"Approaching Left. Appending path back to Right.")
            self.append_path(self._left_node, self._right_node)
            self._last_appended_target = self._right_node

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLinePathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Simple Line Path Publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
