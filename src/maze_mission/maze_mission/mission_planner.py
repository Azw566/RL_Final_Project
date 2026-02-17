import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from aruco_msgs.msg import MarkerArray


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.aruco_sub = self.create_subscription(
            MarkerArray, '/aruco/markers', self.tag_callback, 10
        )

        # Publisher for rescue goals
        self.rescue_goal_pub = self.create_publisher(
            PoseStamped, '/mission/rescue_goal', 10
        )

        # State
        self.detected_marker_ids = set()
        self.last_coverage_pct = 0.0

    def map_callback(self, msg):
        total_cells = msg.info.width * msg.info.height
        if total_cells == 0:
            return
        unknown_cells = msg.data.count(-1)
        known_cells = total_cells - unknown_cells
        coverage_pct = (known_cells / total_cells) * 100.0

        # Log only when coverage changes by at least 1%
        if abs(coverage_pct - self.last_coverage_pct) >= 1.0:
            self.last_coverage_pct = coverage_pct
            self.get_logger().info(
                f'Map coverage: {coverage_pct:.1f}% '
                f'({known_cells}/{total_cells} cells known)'
            )

    def tag_callback(self, msg):
        for marker in msg.markers:
            if marker.id in self.detected_marker_ids:
                continue
            self.detected_marker_ids.add(marker.id)
            self.get_logger().info(
                f'New ArUco marker detected: ID {marker.id}'
            )

            # Publish rescue goal at the marker pose
            goal = PoseStamped()
            goal.header = marker.header
            goal.pose = marker.pose.pose
            self.rescue_goal_pub.publish(goal)

            self.get_logger().info(
                f'Rescue goal published for marker {marker.id} '
                f'at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
