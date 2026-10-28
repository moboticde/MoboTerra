#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import your custom messages and services
from agv_msgs.msg import PathStamped
from roadmap_search_msgs.srv import PlanPathById

from follow_path_msgs.action import FollowPath
from follow_path_msgs.srv import AppendPath


def sanitize_stop_duration(duration_sec):
    """Return a finite, non-negative geometric stop duration."""
    return max(0.0, duration_sec) if math.isfinite(duration_sec) else 0.0


class AgvPathFetcherPublisher(Node):
    def __init__(self):
        super().__init__('agv_path_fetcher_publisher')

        self.declare_parameter('center_node_name', 'center')
        self.declare_parameter('entry_node_name_suffix', 'loop_entry')
        self.declare_parameter('exit_node_name_suffix', 'loop_exit')
        self.declare_parameter('initial_direction', 'cw')
        self.declare_parameter('map_frame_id', 'world')
        self.declare_parameter('motion_mode', 'crab')
        self.declare_parameter('restart_on_goal_result', True)
        self.declare_parameter('stop_before_turn_enabled', True)
        self.declare_parameter('stop_node_names', ['cw_loop_entry', 'ccw_loop_exit'])
        self.declare_parameter('stop_duration_sec', 8.0)

        self._center_node_name = self.get_parameter('center_node_name').get_parameter_value().string_value
        self._entry_node_name_suffix = self.get_parameter('entry_node_name_suffix').get_parameter_value().string_value
        self._exit_node_name_suffix = self.get_parameter('exit_node_name_suffix').get_parameter_value().string_value
        self._initial_direction = self.get_parameter('initial_direction').get_parameter_value().string_value
        self._map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self._motion_mode = self.get_parameter('motion_mode').get_parameter_value().string_value
        self._restart_on_goal_result = self.get_parameter('restart_on_goal_result').get_parameter_value().bool_value
        self._stop_before_turn_enabled = (
            self.get_parameter('stop_before_turn_enabled').get_parameter_value().bool_value
        )
        self._stop_node_names = set(
            self.get_parameter('stop_node_names').get_parameter_value().string_array_value
        )
        self._stop_duration_sec = self.get_parameter('stop_duration_sec').get_parameter_value().double_value

        self._stop_timer = None
        self._stop_restart_pending = None

        self.sub_active_segments_ = self.create_subscription(PathStamped, 'active_segments', self.cb_active_segments, 1)

        self._cl_plan_path = self.create_client(PlanPathById, 'get_shortest_path_by_id')
        self._cl_path_append = self.create_client(AppendPath, 'append_path')
        self._cl_path_follow = ActionClient(self, FollowPath, 'path_follower')

        while not self._cl_plan_path.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "get_shortest_path_by_id" service...')

        while not self._cl_path_append.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "append_path" service...')

        while not self._cl_path_follow.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for "follow_path" action server...')

        # Simplified state variables
        self._last_appended = None
        self._last_active_end = None
        self._pending_append_target = None
        self._goal_active = False
        self._current_goal_end = None
        self._path_cache = dict()

        self.send_initial_goal()

    def loop_nodes(self):
        entry_val = 0 if self._initial_direction == 'cw' else -18000
        full_entry_name = f"{self._initial_direction}_{self._entry_node_name_suffix}_{entry_val}"
        halfway_name = f"{'ccw' if self._initial_direction == 'cw' else 'cw'}_{self._exit_node_name_suffix}_-18000"
        return full_entry_name, halfway_name

    def next_loop_target(self, current_node):
        full_entry_name, halfway_name = self.loop_nodes()
        if current_node == full_entry_name:
            return halfway_name
        if current_node == halfway_name:
            return full_entry_name
        return None

    def is_stop_node(self, node_name):
        return any(
            node_name == stop_node_name or node_name.startswith(f'{stop_node_name}_')
            for stop_node_name in self._stop_node_names
        )

    def effective_stop_duration(self):
        return sanitize_stop_duration(self._stop_duration_sec)

    def send_path_as_goal(self, path):
        if not path.segments:
            self.get_logger().error('Cannot send empty path as a FollowPath goal.')
            return

        path_stamped = PathStamped()
        path_stamped.header.stamp = self.get_clock().now().to_msg()
        path_stamped.header.frame_id = self._map_frame_id
        path_stamped.path = path

        for segment in path_stamped.path.segments:
            segment.type = f"type={self._motion_mode};"

        goal_msg = FollowPath.Goal()
        goal_msg.path_stamped = path_stamped

        self.get_logger().info(
            f"Sending goal from {path.segments[0].begin.id} to {path.segments[-1].end.id}."
        )
        self._current_goal_end = path.segments[-1].end.id
        self._last_active_end = self._current_goal_end
        future = self._cl_path_follow.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self._goal_active = False
            self.get_logger().error(f'Goal request failed: {e}')
            return

        if not goal_handle.accepted:
            self._goal_active = False
            self.get_logger().error('FollowPath goal was rejected.')
            return

        self._goal_active = True
        self.get_logger().info('FollowPath goal was accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self._goal_active = False
        try:
            response = future.result()
            success = response.result.success
            status = response.status
        except Exception as e:
            self.get_logger().error(f'Goal result failed: {e}')
            return

        self.get_logger().warn(f'FollowPath goal finished: success={success}, status={status}.')
        if self._restart_on_goal_result and success:
            self.restart_loop_after_result()

    def restart_loop_after_result(self):
        begin_node = self._last_active_end or self._current_goal_end
        end_node = self.next_loop_target(begin_node)

        if end_node is None:
            self.get_logger().warn(
                f'Cannot restart from last active endpoint "{begin_node}". Sending initial loop goal.'
            )
            self.send_initial_goal()
            return

        if self._stop_before_turn_enabled and self.is_stop_node(begin_node):
            self.schedule_loop_restart_after_stop(begin_node, end_node)
            return

        self.restart_loop_goal(begin_node, end_node)

    def restart_loop_goal(self, begin_node, end_node):
        self.get_logger().warn(f'Restarting loop with new goal from {begin_node} to {end_node}.')
        self._last_appended = end_node
        self._pending_append_target = None
        self.request_plan(begin_node, end_node, self.send_future_as_goal)

    def schedule_loop_restart_after_stop(self, begin_node, end_node):
        if self._stop_timer is not None:
            self.get_logger().debug(
                f'Stop timer already active at {begin_node}. Not scheduling another restart.'
            )
            return

        self._stop_restart_pending = (begin_node, end_node)
        stop_duration_sec = self.effective_stop_duration()

        if stop_duration_sec <= 0.0:
            self.stop_timer_callback()
            return

        self.get_logger().warn(
            f'Stopped at {begin_node}. Waiting {stop_duration_sec:.1f} seconds before continuing to {end_node}.'
        )
        self._stop_timer = self.create_timer(stop_duration_sec, self.stop_timer_callback)

    def stop_timer_callback(self):
        if self._stop_timer is not None:
            stop_timer = self._stop_timer
            self._stop_timer = None
            stop_timer.cancel()
            self.destroy_timer(stop_timer)

        if self._stop_restart_pending is None:
            return

        begin_node, end_node = self._stop_restart_pending
        self._stop_restart_pending = None
        self.restart_loop_goal(begin_node, end_node)

    def send_initial_goal(self):
        entry_val = 0 if self._initial_direction == 'cw' else -18000
        begin_node = f"{self._center_node_name}_{entry_val}"
        end_node = f"{self._initial_direction}_{self._entry_node_name_suffix}_{entry_val}"

        # Set the initial last_appended to the end of the first goal
        self._last_appended = end_node

        if (begin_node, end_node) in self._path_cache:
            self.get_logger().info("Found path from begin_node to end_node in cache. Sending it as an initial goal!")
            self.send_path_as_goal(self._path_cache[(begin_node, end_node)])
            return

        self.get_logger().info(f"Requesting planning from {begin_node} to {end_node}. Path isn't in cache.")
        self.request_plan(begin_node, end_node, self.send_future_as_goal)

    def send_path_to_append(self, path, expected_end_node=None):
        if not path.segments:
            self.get_logger().error('Cannot append empty path.')
            self._pending_append_target = None
            return

        path_stamped = PathStamped()
        path_stamped.header.stamp = self.get_clock().now().to_msg()
        path_stamped.header.frame_id = self._map_frame_id
        path_stamped.path = path

        for segment in path_stamped.path.segments:
            segment.type = f"type={self._motion_mode};"

        request = AppendPath.Request()
        request.path = path_stamped
        future = self._cl_path_append.call_async(request)
        future.add_done_callback(
            lambda f: self.append_response_callback(f, expected_end_node or path.segments[-1].end.id)
        )

    def append_response_callback(self, future, expected_end_node):
        try:
            response = future.result()
        except Exception as e:
            self._pending_append_target = None
            self.get_logger().error(f'AppendPath service call failed: {e}')
            return

        self._pending_append_target = None
        if response.success:
            self._last_appended = expected_end_node
            self.get_logger().info(f'AppendPath accepted. Last appended endpoint is now {expected_end_node}.')
        else:
            self.get_logger().error(
                f'AppendPath rejected for endpoint {expected_end_node}. Will retry on the next active_segments update.'
            )

    def request_plan(self, begin_node, end_node, callback):
        request = PlanPathById.Request()
        request.node_start = begin_node
        request.node_end = end_node

        future = self._cl_plan_path.call_async(request)
        future.add_done_callback(lambda f: callback(f, begin_node, end_node))

    def send_future_to_append(self, future, begin_node, end_node):
        try:
            path = future.result().path
            if not path.segments:
                self._pending_append_target = None
                self.get_logger().error(f'Planner returned empty append path from {begin_node} to {end_node}.')
                return

            self._path_cache[(path.segments[0].begin.id, path.segments[-1].end.id)] = path
            self.send_path_to_append(path, end_node)
        except Exception as e:
            self._pending_append_target = None
            self.get_logger().error(f'Append service call failed: {e}. Cannot send path to append to PF!')

    def send_future_as_goal(self, future, begin_node, end_node):
        try:
            path = future.result().path
            if not path.segments:
                self.get_logger().error(f'Planner returned empty goal path from {begin_node} to {end_node}.')
                return

            self._path_cache[(path.segments[0].begin.id, path.segments[-1].end.id)] = path
            self.send_path_as_goal(path)
        except Exception as e:
            self.get_logger().error(f'Planning service call failed: {e}. Cannot send goal path to follower.')

    def feedback_callback(self, feedback_msg):
        self.get_logger().debug(f'Received feedback: {feedback_msg}')

    def append_path(self, begin_node, end_node):
        if self._pending_append_target is not None:
            self.get_logger().debug(
                f'Append to {self._pending_append_target} is already pending. Not sending another append.'
            )
            return

        self._pending_append_target = end_node

        if (begin_node, end_node) in self._path_cache:
            self.get_logger().info("Found path from begin_node to end_node in cache. Sending it to append!")
            self.send_path_to_append(self._path_cache[(begin_node, end_node)], end_node)
            return

        self.get_logger().info(f"Requesting planning from {begin_node} to {end_node}. Path isn't in cache.")
        self.request_plan(begin_node, end_node, self.send_future_to_append)

    def cb_active_segments(self, msg):
        if not msg.path.segments:
            return

        curr_seg_begin = msg.path.segments[0].begin.id
        curr_seg_end = msg.path.segments[-1].end.id
        self._last_active_end = curr_seg_end

        self.get_logger().info(f"On segment from {curr_seg_begin} to {curr_seg_end}:")

        # Only trigger append logic if the active segment ends at the node we last appended
        if curr_seg_end != self._last_appended:
            return

        if self._stop_before_turn_enabled and self.is_stop_node(self._last_appended):
            self.get_logger().debug(
                f'Endpoint {self._last_appended} is configured as a stop node. Waiting for goal result before continuing.'
            )
            return

        full_entry_name, halfway_name = self.loop_nodes()

        # Ping-pong between full_entry_name and halfway_name indefinitely
        if self._last_appended == full_entry_name:
            self.get_logger().info(f"\tRequesting append from {full_entry_name} to {halfway_name} (Indefinite Loop)")
            self.append_path(full_entry_name, halfway_name)

        elif self._last_appended == halfway_name:
            self.get_logger().info(f"\tRequesting append from {halfway_name} to {full_entry_name} (Indefinite Loop)")
            self.append_path(halfway_name, full_entry_name)


def main(args=None):
    rclpy.init(args=args)
    node = AgvPathFetcherPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
