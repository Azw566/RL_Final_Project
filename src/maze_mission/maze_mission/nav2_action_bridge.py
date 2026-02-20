#!/usr/bin/env python3
"""
nav2_action_bridge.py — Fake Nav2 action server for PX4 drones.

Exposes a NavigateToPose action server so that explore_lite can send goals
to the drone as if it were a Nav2-managed ground robot.

Flow:
  1. Arms the drone via VehicleCommand
  2. Sends takeoff command and waits for airborne state
  3. On NavigateToPose goal (ENU coordinates from explore_lite):
       Converts ENU → NED and streams TrajectorySetpoint + OffboardControlMode
  4. Monitors /odom (ENU, from odom_converter) for convergence
  5. Returns SUCCEEDED when within threshold of goal

Parameters (ROS 2):
    altitude   — target flight altitude in meters (default 1.2m — mid-wall)
    threshold  — goal-reached distance in meters (default 0.5m)
    ns         — drone namespace (e.g. "drone1")
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)


class Nav2ActionBridge(Node):

    STATE_IDLE = 'IDLE'
    STATE_ARMING = 'ARMING'
    STATE_TAKING_OFF = 'TAKING_OFF'
    STATE_FLYING = 'FLYING'
    STATE_SUCCEEDED = 'SUCCEEDED'

    def __init__(self):
        super().__init__('nav2_action_bridge')

        self.declare_parameter('altitude', 1.2)
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('ns', '')

        self.altitude = self.get_parameter('altitude').value
        self.threshold = self.get_parameter('threshold').value
        ns = self.get_parameter('ns').value
        self.ns = ns if ns else ''

        # PX4 QoS — BEST_EFFORT + VOLATILE to match micro-xrce-dds settings
        # (odom_converter uses qos_profile_sensor_data which is also VOLATILE)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers to PX4
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'/{self.ns}/fmu/in/offboard_control_mode' if self.ns else '/fmu/in/offboard_control_mode',
            px4_qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/{self.ns}/fmu/in/trajectory_setpoint' if self.ns else '/fmu/in/trajectory_setpoint',
            px4_qos,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            f'/{self.ns}/fmu/in/vehicle_command' if self.ns else '/fmu/in/vehicle_command',
            px4_qos,
        )

        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'/{self.ns}/fmu/out/vehicle_status' if self.ns else '/fmu/out/vehicle_status',
            self.status_callback,
            px4_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.ns}/odom' if self.ns else '/odom',
            self.odom_callback,
            10,
        )

        # Action server — expose navigate_to_pose under namespace
        action_name = f'/{self.ns}/navigate_to_pose' if self.ns else '/navigate_to_pose'
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # State
        self.state = self.STATE_IDLE
        self.vehicle_status = None
        self.current_pos_enu = None   # (x, y, z) ENU
        self.goal_pos_enu = None      # (x, y) ENU from explore_lite
        self.setpoint_timer = None
        self.arm_attempt_count = 0

        # Heartbeat timer (keep offboard alive at 10 Hz)
        self.heartbeat_timer = self.create_timer(0.1, self.heartbeat_callback)

        self.get_logger().info(f'Nav2ActionBridge ready (ns={self.ns}, alt={self.altitude}m)')

    # ─── Subscribers ───────────────────────────────────────────────────────────

    def status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_pos_enu = (p.x, p.y, p.z)

    # ─── Action server callbacks ────────────────────────────────────────────────

    def goal_callback(self, goal_request):
        self.get_logger().info('NavigateToPose goal received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('NavigateToPose cancel requested')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing NavigateToPose goal')

        pose: PoseStamped = goal_handle.request.pose
        self.goal_pos_enu = (pose.pose.position.x, pose.pose.position.y)
        self.get_logger().info(
            f'Goal ENU: ({self.goal_pos_enu[0]:.2f}, {self.goal_pos_enu[1]:.2f})'
        )

        # Step 1: Arm if not already armed
        await self._ensure_armed(goal_handle)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return NavigateToPose.Result()

        # Step 2: Take off if on ground
        await self._ensure_airborne(goal_handle)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return NavigateToPose.Result()

        # Step 3: Fly to goal
        self.state = self.STATE_FLYING
        result = await self._fly_to_goal(goal_handle)

        if result:
            goal_handle.succeed()
            self.get_logger().info('NavigateToPose SUCCEEDED')
        else:
            goal_handle.canceled()

        self.state = self.STATE_IDLE
        self.goal_pos_enu = None
        return NavigateToPose.Result()

    # ─── Flight helpers ─────────────────────────────────────────────────────────

    async def _ensure_armed(self, goal_handle):
        """Arm the drone if not already armed."""
        self.state = self.STATE_ARMING
        self.get_logger().info('Arming drone...')

        import asyncio
        self.arm_attempt_count = 0
        while True:
            if goal_handle.is_cancel_requested:
                return
            if self.vehicle_status is not None and self.vehicle_status.arming_state == 2:
                self.get_logger().info('Drone armed')
                return
            if self.arm_attempt_count % 10 == 0:
                self._publish_arm_command()
            self.arm_attempt_count += 1
            await asyncio.sleep(0.1)

    async def _ensure_airborne(self, goal_handle):
        """Take off and wait until at target altitude."""
        self.state = self.STATE_TAKING_OFF
        self.get_logger().info(f'Taking off to {self.altitude}m...')

        import asyncio
        attempt = 0
        while True:
            if goal_handle.is_cancel_requested:
                return
            if attempt % 10 == 0:
                self._publish_offboard_mode()
                self._publish_takeoff_setpoint()
                if attempt == 0:
                    self._publish_offboard_command()
            attempt += 1

            if self.current_pos_enu is not None:
                alt = self.current_pos_enu[2]
                if alt >= self.altitude * 0.9:
                    self.get_logger().info(f'Airborne at {alt:.2f}m')
                    return
            await asyncio.sleep(0.1)

    async def _fly_to_goal(self, goal_handle):
        """Stream setpoints toward the goal. Returns True on success."""
        import asyncio
        goal_x, goal_y = self.goal_pos_enu
        # Convert ENU goal to NED for PX4
        # x_ned = y_enu, y_ned = x_enu, z_ned = -altitude
        ned_x = goal_y
        ned_y = goal_x
        ned_z = -self.altitude

        self.get_logger().info(f'Flying to NED ({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f})')

        timeout = 120.0  # seconds
        elapsed = 0.0
        dt = 0.1

        while elapsed < timeout:
            if goal_handle.is_cancel_requested:
                return False

            self._publish_offboard_mode()
            self._publish_position_setpoint(ned_x, ned_y, ned_z)

            if self.current_pos_enu is not None:
                dx = self.current_pos_enu[0] - goal_x
                dy = self.current_pos_enu[1] - goal_y
                dist = math.sqrt(dx * dx + dy * dy)

                # Publish feedback
                feedback = NavigateToPose.Feedback()
                feedback.distance_remaining = dist
                goal_handle.publish_feedback(feedback)

                if dist < self.threshold:
                    self.get_logger().info(f'Goal reached (dist={dist:.3f}m)')
                    return True

            await asyncio.sleep(dt)
            elapsed += dt

        self.get_logger().warn('Timeout flying to goal')
        return False

    # ─── Heartbeat (keeps offboard mode alive) ──────────────────────────────────

    def heartbeat_callback(self):
        if self.state in (self.STATE_FLYING, self.STATE_TAKING_OFF):
            self._publish_offboard_mode()

    # ─── PX4 command helpers ────────────────────────────────────────────────────

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        self.offboard_pub.publish(msg)

    def _publish_position_setpoint(self, x_ned, y_ned, z_ned):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x_ned), float(y_ned), float(z_ned)]
        msg.yaw = float('nan')
        self.setpoint_pub.publish(msg)

    def _publish_takeoff_setpoint(self):
        self._publish_position_setpoint(0.0, 0.0, -self.altitude)

    def _publish_arm_command(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _publish_offboard_command(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
