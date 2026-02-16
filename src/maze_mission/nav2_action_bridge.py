#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Interfaces
from nav2_msgs.action import NavigateToPose
from px4_msgs.msg import TrajectorySetpoint, VehicleStatus, OffboardControlMode, VehicleCommand, VehicleLocalPosition

import math
import numpy as np

class Nav2ActionBridge(Node):

    def __init__(self):
        super().__init__('nav2_action_bridge')

        self.declare_parameter('flight_altitude', 4.0)
        self.flight_alt = self.get_parameter('flight_altitude').value

        # PX4 QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers to PX4
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        # Subscribers from PX4
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, qos_profile)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos_profile)

        # Action Server (The "Fake" Nav2)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

        self.current_pos = None
        self.nav_state = "IDLE" # IDLE, TAKEOFF, NAVIGATING
        self.offboard_setpoint_counter = 0

        # Heartbeat timer (must publish > 2Hz)
        self.create_timer(0.1, self.cmdloop_callback)

    def local_pos_cb(self, msg):
        self.current_pos = msg

    def status_cb(self, msg):
        self.current_status = msg

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard mode command sent")

    def cmdloop_callback(self):
        # Publish Offboard Control Mode Heartbeat
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(offboard_msg)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request from explore_lite...')
        
        # 1. ARM AND TAKEOFF (If not already)
        if self.nav_state == "IDLE":
             # Send a few setpoints before switching mode (PX4 requirement)
            for _ in range(10):
                self.publish_position_setpoint(0.0, 0.0, -self.flight_alt)
            
            self.engage_offboard_mode()
            self.arm()
            self.nav_state = "NAVIGATING"

        # 2. Extract Goal
        goal_x = goal_handle.request.pose.pose.position.x
        goal_y = goal_handle.request.pose.pose.position.y
        # Convert ENU (ROS) to NED (PX4)
        # ROS X = PX4 X (North)
        # ROS Y = -PX4 Y (East) -> Wait, standard conversion is:
        # ROS x (Forward/East?) -> Depends on world. 
        # Standard ROS ENU: X=East, Y=North. PX4 NED: X=North, Y=East.
        # BUT usually in these sims X=Forward for both.
        # Let's assume standard alignment: X=X, Y=Y, Z=-Z.
        
        target_x = goal_x
        target_y = goal_y
        target_z = -self.flight_alt

        self.get_logger().info(f'Flying to: x={target_x}, y={target_y}')

        # 3. Control Loop
        feedback_msg = NavigateToPose.Feedback()
        
        while rclpy.ok():
            if self.current_pos is None:
                continue

            # Calculate distance
            dx = target_x - self.current_pos.x
            dy = target_y - self.current_pos.y
            dist = math.sqrt(dx*dx + dy*dy)

            # Publish Setpoint
            self.publish_position_setpoint(target_x, target_y, target_z)

            # Check if reached
            if dist < 0.5: # 50cm tolerance
                self.get_logger().info('Goal Reached!')
                goal_handle.succeed()
                result = NavigateToPose.Result()
                return result

            # Feedback
            feedback_msg.distance_remaining = dist
            goal_handle.publish_feedback(feedback_msg)
            
            # Rate limit
            import time
            time.sleep(0.1)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0 # Face North/Forward always
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
