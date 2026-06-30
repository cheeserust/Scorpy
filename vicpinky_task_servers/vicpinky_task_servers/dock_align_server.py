#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vicpinky_interfaces.action import RunTask


class DockAlignServer(Node):
    def __init__(self):
        super().__init__('dock_align_server')

        self.declare_parameter('server_name', '/dock/align')
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('mock_delay_sec', 2.0)

        self.server_name = self.get_parameter('server_name').value
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.mock_delay_sec = float(self.get_parameter('mock_delay_sec').value)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.action_server = ActionServer(
            self,
            RunTask,
            self.server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Dock Align Action Server Started.')
        self.get_logger().info(f'server_name: {self.server_name}')
        self.get_logger().info(f'mock_mode: {self.mock_mode}')

    def goal_callback(self, goal_request):
        self.get_logger().info('')
        self.get_logger().info('========== /dock/align GOAL ==========')
        self.get_logger().info(f'task_id      : {goal_request.task_id}')
        self.get_logger().info(f'target_name  : {goal_request.target_name}')
        self.get_logger().info(f'target_floor : {goal_request.target_floor}')
        self.get_logger().info(f'marker_id    : {goal_request.marker_id}')
        self.get_logger().info('======================================')

        if goal_request.task_id not in ('dock_to_marker', 'align', 'tag_align'):
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.stop_robot()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = RunTask.Result()

        phases = [
            ('SEARCH_MARKER', 0.25, f'search marker {goal.marker_id}'),
            ('ROTATE_ALIGN', 0.50, 'aligning heading'),
            ('DISTANCE_ALIGN', 0.75, 'aligning distance'),
            ('ALIGNED', 1.00, 'marker aligned'),
        ]

        for phase, progress, detail in phases:
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                result.success = False
                result.message = 'dock align canceled'
                goal_handle.canceled()
                return result

            feedback = RunTask.Feedback()
            feedback.phase = phase
            feedback.progress = progress
            feedback.detail = detail
            goal_handle.publish_feedback(feedback)

            if not self.mock_mode:
                cmd = Twist()
                cmd.angular.z = 0.08 if phase == 'ROTATE_ALIGN' else 0.0
                cmd.linear.x = 0.03 if phase == 'DISTANCE_ALIGN' else 0.0
                self.cmd_vel_pub.publish(cmd)

            time.sleep(self.mock_delay_sec / len(phases))

        self.stop_robot()
        result.success = True
        result.message = f'aligned to marker {goal.marker_id}'
        goal_handle.succeed()
        return result

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DockAlignServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
