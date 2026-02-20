#!/usr/bin/env python3
"""
turtlebot_controller.py — Receives retrieval goals from mission_planner,
sends them as NavigateToPose actions to the TurtleBot's Nav2 stack.

Subscribes: /mission/turtlebot_goal (geometry_msgs/PoseStamped)
Action client: /navigate_to_pose (nav2_msgs/NavigateToPose)

The goals arrive in the map frame (ENU), which is the same frame the
TurtleBot's Nav2 is operating in. No coordinate conversion needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/mission/turtlebot_goal',
            self.goal_callback,
            10
        )

        self.current_goal_handle = None
        self.get_logger().info('TurtleBotController: waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('TurtleBotController: Nav2 action server available')

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

        if self.current_goal_handle is not None:
            self.get_logger().info('Cancelling previous goal')
            self.current_goal_handle.cancel_goal_async()

        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        if remaining < 1.0:
            self.get_logger().info(f'Distance remaining: {remaining:.2f}m')

    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation SUCCEEDED — at target')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
        self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
