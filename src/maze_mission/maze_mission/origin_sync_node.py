#!/usr/bin/env python3
"""
origin_sync_node.py — Publishes /initialpose for TurtleBot AMCL.

Concept: The TurtleBot's physical starting position (in the real world or
Gazebo) must be expressed as coordinates in the drone's map frame. Since
odom_converter has already transformed everything to ENU, and slam_toolbox
builds the map in ENU, the map frame is ENU-aligned. We simply provide the
TurtleBot's spawn coordinates in that frame.

Parameters:
    initial_x, initial_y, initial_yaw — TurtleBot's start pose in map frame
    publish_count — how many times to publish (AMCL might miss the first one)
    publish_interval — seconds between publishes
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class OriginSyncNode(Node):
    def __init__(self):
        super().__init__('origin_sync_node')

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('publish_interval', 1.0)

        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.yaw = self.get_parameter('initial_yaw').value
        self.max_count = self.get_parameter('publish_count').value
        self.interval = self.get_parameter('publish_interval').value

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.count = 0
        self.timer = self.create_timer(self.interval, self.publish_initial_pose)

        self.get_logger().info(
            f'OriginSync: will publish initialpose at '
            f'({self.x}, {self.y}, yaw={self.yaw}) {self.max_count} times'
        )

    def publish_initial_pose(self):
        if self.count >= self.max_count:
            self.timer.cancel()
            self.get_logger().info('OriginSync: done publishing initialpose')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # 6x6 covariance matrix (flattened), diagonal only
        cov = [0.0] * 36
        cov[0] = 0.25   # x variance (0.5m std dev)
        cov[7] = 0.25   # y variance
        cov[35] = 0.07  # yaw variance (~15° std dev)
        msg.pose.covariance = cov

        self.publisher.publish(msg)
        self.count += 1
        self.get_logger().info(f'OriginSync: published initialpose ({self.count}/{self.max_count})')


def main(args=None):
    rclpy.init(args=args)
    node = OriginSyncNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
