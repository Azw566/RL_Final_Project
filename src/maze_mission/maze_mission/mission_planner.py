#!/usr/bin/env python3
"""
mission_planner.py — Central mission coordinator.

State machine:
  EXPLORING  — Drones are mapping the maze; AMCL waits
  RETRIEVING — Tag found; TurtleBot dispatched to retrieve
  COMPLETE   — All tags retrieved (or exploration done)

Responsibilities:
  - Monitor /map for exploration progress
  - Collect ArUco detections from /aruco/detections
  - Dispatch TurtleBot retrieval goals via /mission/turtlebot_goal
  - Pause/resume explore_lite on each drone
  - Publish mission status to /mission/status

Parameters:
    exploration_coverage_threshold — fraction [0,1] of map cells known before
                                     triggering retrieval (default 0.6)
    max_tags                       — total expected ArUco tags (default 5)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from maze_msgs.msg import ArucoDetectionArray


class MissionPlanner(Node):

    STATE_EXPLORING = 'EXPLORING'
    STATE_RETRIEVING = 'RETRIEVING'
    STATE_COMPLETE = 'COMPLETE'

    def __init__(self):
        super().__init__('mission_planner')

        self.declare_parameter('exploration_coverage_threshold', 0.60)
        self.declare_parameter('max_tags', 5)
        self.declare_parameter('drone_namespaces', ['drone1', 'drone2'])
        self.declare_parameter('retrieval_complete_distance', 0.5)

        self.coverage_threshold = self.get_parameter('exploration_coverage_threshold').value
        self.max_tags = self.get_parameter('max_tags').value
        self.drone_ns = self.get_parameter('drone_namespaces').value
        self.retrieval_done_dist = self.get_parameter('retrieval_complete_distance').value

        # State
        self.state = self.STATE_EXPLORING
        self.detected_tags = {}         # marker_id → PoseStamped in map frame
        self.pending_tags = []          # queue of marker_ids to retrieve
        self.retrieved_tags = set()     # marker_ids done
        self.active_retrieval_id = None
        self.coverage = 0.0

        # Map monitoring
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # ArUco detections
        self.aruco_sub = self.create_subscription(
            ArucoDetectionArray, '/aruco/detections', self.aruco_callback, 10
        )

        # TurtleBot goal dispatcher
        self.goal_pub = self.create_publisher(PoseStamped, '/mission/turtlebot_goal', 10)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # explore_lite pause/resume (one per drone)
        self.explore_pause_pubs = {}
        for ns in self.drone_ns:
            pub = self.create_publisher(Bool, f'/{ns}/explore/resume', 10)
            self.explore_pause_pubs[ns] = pub

        # Periodic logic at 2 Hz
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info(
            f'MissionPlanner ready (coverage_thresh={self.coverage_threshold}, '
            f'max_tags={self.max_tags})'
        )

    # ─── Subscribers ───────────────────────────────────────────────────────────

    def map_callback(self, msg: OccupancyGrid):
        total_cells = msg.info.width * msg.info.height
        if total_cells == 0:
            return
        known = sum(1 for c in msg.data if c >= 0)
        self.coverage = known / total_cells

    def aruco_callback(self, msg: ArucoDetectionArray):
        for det in msg.detections:
            mid = det.marker_id
            if mid not in self.detected_tags:
                self.get_logger().info(
                    f'New ArUco tag detected: ID {mid} at '
                    f'({det.pose.pose.position.x:.2f}, {det.pose.pose.position.y:.2f})'
                )
                self.detected_tags[mid] = det.pose
                if mid not in self.retrieved_tags and mid not in self.pending_tags:
                    self.pending_tags.append(mid)

    # ─── Main state machine tick ────────────────────────────────────────────────

    def tick(self):
        if self.state == self.STATE_EXPLORING:
            self._tick_exploring()
        elif self.state == self.STATE_RETRIEVING:
            self._tick_retrieving()
        elif self.state == self.STATE_COMPLETE:
            self._tick_complete()

        self._publish_status()

    def _tick_exploring(self):
        # Transition to RETRIEVING if enough of the map is known OR all tags found
        should_retrieve = (
            self.pending_tags and (
                self.coverage >= self.coverage_threshold
                or len(self.detected_tags) >= self.max_tags
            )
        )
        if should_retrieve:
            self.get_logger().info(
                f'Coverage {self.coverage:.1%} — transitioning to RETRIEVING'
            )
            self._pause_exploration()
            self.state = self.STATE_RETRIEVING
            self._dispatch_next_tag()

    def _tick_retrieving(self):
        if self.active_retrieval_id is None:
            if self.pending_tags:
                self._dispatch_next_tag()
            else:
                # No more tags — check if we're done
                if len(self.retrieved_tags) >= self.max_tags:
                    self.state = self.STATE_COMPLETE
                    self.get_logger().info('All tags retrieved — MISSION COMPLETE')
                else:
                    # Resume exploration to find more tags
                    self.get_logger().info(
                        'No pending tags — resuming exploration to find more'
                    )
                    self._resume_exploration()
                    self.state = self.STATE_EXPLORING

    def _tick_complete(self):
        pass  # Mission done; could trigger landing here

    # ─── Helpers ────────────────────────────────────────────────────────────────

    def _dispatch_next_tag(self):
        if not self.pending_tags:
            return
        mid = self.pending_tags.pop(0)
        pose = self.detected_tags.get(mid)
        if pose is None:
            self.get_logger().warn(f'No pose for tag {mid}')
            return

        self.active_retrieval_id = mid
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose = pose.pose
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'Dispatching TurtleBot to tag {mid} at '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )

        # Simple timeout: mark as retrieved after 60s (Nav2 result handled by controller)
        self.create_timer(60.0, lambda: self._mark_retrieved(mid))

    def _mark_retrieved(self, marker_id: int):
        if self.active_retrieval_id == marker_id:
            self.retrieved_tags.add(marker_id)
            self.active_retrieval_id = None
            self.get_logger().info(f'Tag {marker_id} marked as retrieved')

    def _pause_exploration(self):
        msg = Bool()
        msg.data = False
        for ns, pub in self.explore_pause_pubs.items():
            pub.publish(msg)
        self.get_logger().info('Exploration PAUSED')

    def _resume_exploration(self):
        msg = Bool()
        msg.data = True
        for ns, pub in self.explore_pause_pubs.items():
            pub.publish(msg)
        self.get_logger().info('Exploration RESUMED')

    def _publish_status(self):
        msg = String()
        msg.data = (
            f'state={self.state} '
            f'coverage={self.coverage:.1%} '
            f'detected={list(self.detected_tags.keys())} '
            f'retrieved={list(self.retrieved_tags)} '
            f'pending={self.pending_tags}'
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
