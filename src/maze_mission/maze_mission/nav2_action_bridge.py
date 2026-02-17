import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleLocalPosition,
    OffboardControlMode,
)


class Nav2ActionBridge(Node):
    def __init__(self):
        super().__init__('nav2_action_bridge')

        # Parameters
        self.declare_parameter('namespace', 'drone1')
        self.declare_parameter('target_altitude', 4.0)
        self.declare_parameter('position_tolerance', 0.5)
        ns = self.get_parameter('namespace').value
        self.target_altitude = self.get_parameter('target_altitude').value
        self.position_tolerance = self.get_parameter('position_tolerance').value

        # Action Server for explore_lite / Nav2
        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self.execute_callback
        )

        # PX4 Interface - QoS must be BEST_EFFORT for PX4 micro-XRCE-DDS
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pose_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/{ns}/fmu/out/vehicle_local_position',
            self.pose_cb,
            px4_qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'/{ns}/fmu/in/trajectory_setpoint', 10
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f'/{ns}/fmu/in/offboard_control_mode', 10
        )

        # 10 Hz heartbeat to keep offboard mode active
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.current_pose = None

    def pose_cb(self, msg):
        self.current_pose = msg

    def timer_cb(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    async def execute_callback(self, goal_handle):
        target = goal_handle.request.pose.pose.position
        self.get_logger().info(
            f'Target received: ENU({target.x:.2f}, {target.y:.2f})'
        )

        # ENU -> NED: swap x<->y, negate z
        ned_x = target.y
        ned_y = target.x
        ned_z = -self.target_altitude

        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok():
            # Publish setpoint
            setpoint = TrajectorySetpoint()
            setpoint.position = [float(ned_x), float(ned_y), float(ned_z)]
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.setpoint_pub.publish(setpoint)

            # Publish feedback
            if self.current_pose is not None:
                dx = ned_x - self.current_pose.x
                dy = ned_y - self.current_pose.y
                distance = math.sqrt(dx * dx + dy * dy)

                feedback = NavigateToPose.Feedback()
                feedback.distance_remaining = float(distance)
                goal_handle.publish_feedback(feedback)

                if distance < self.position_tolerance:
                    self.get_logger().info('Goal reached')
                    goal_handle.succeed()
                    return NavigateToPose.Result()

            rate.sleep()

        # If rclpy shut down, abort
        goal_handle.abort()
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
