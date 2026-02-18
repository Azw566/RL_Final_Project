import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleLocalPosition,
    OffboardControlMode,
    VehicleCommand,
    VehicleStatus
)

class Nav2ActionBridge(Node):
    def __init__(self):
        super().__init__('nav2_action_bridge')

        # --- Paramètres ---
        self.declare_parameter('namespace', 'drone1')
        self.declare_parameter('system_id', 1) # <--- AJOUTE ÇA (1 pour drone1, 2 pour drone2, etc.)
        self.declare_parameter('target_altitude', 2.0)
        
        self.ns = self.get_parameter('namespace').value
        self.sys_id = self.get_parameter('system_id').value
        self.target_alt = self.get_parameter('target_altitude').value
        
        # ... (reste du code identique jusqu'au timer)
        # --- État interne ---
        self.current_pose = None
        self.nav_state = None
        self.arm_state = None
        self.offboard_count = 0
        # On définit une consigne par défaut (maintien de position au sol au début)
        self.active_setpoint = [0.0, 0.0, 0.0] 

        # --- QoS PX4 ---
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Pub/Sub ---
        self.status_sub = self.create_subscription(VehicleStatus, f'/{self.ns}/fmu/out/vehicle_status', self.status_cb, px4_qos)
        self.pose_sub = self.create_subscription(VehicleLocalPosition, f'/{self.ns}/fmu/out/vehicle_local_position', self.pose_cb, px4_qos)
        
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, f'/{self.ns}/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, f'/{self.ns}/fmu/in/offboard_control_mode', 10)
        self.command_pub = self.create_publisher(VehicleCommand, f'/{self.ns}/fmu/in/vehicle_command', 10)

        # --- Action Server ---
        self._action_server = ActionServer(
            self, NavigateToPose, f'/{self.ns}/navigate_to_pose', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
        )

        # Heartbeat 10Hz
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info(f"Bridge Action Nav2 -> PX4 prêt pour {self.ns}")

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state

    def pose_cb(self, msg):
        self.current_pose = msg

    def timer_cb(self):
        """ Publie le heartbeat et force l'état Offboard/Armed """
        # 1. Publier le mode Offboard (Indispensable pour PX4)
        off_msg = OffboardControlMode()
        off_msg.position = True
        off_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(off_msg)

        # 2. Publier la consigne (Setpoint)
        sp_msg = TrajectorySetpoint()
        sp_msg.position = self.active_setpoint
        sp_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(sp_msg)

        # 3. Logique d'armement persistante
        # nav_state 14 = OFFBOARD | arming_state 2 = ARMED
        if self.nav_state is not None:
            if self.nav_state != 14:
                self.get_logger().info(f"[{self.ns}] Tentative passage en OFFBOARD...", once=True)
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            
            elif self.arm_state != 2:
                self.get_logger().info(f"[{self.ns}] Tentative ARMEMENT...", once=True)
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def send_cmd(self, command, p1, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = self.sys_id # <--- Utilise l'ID dynamique ici
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)
    async def execute_callback(self, goal_handle):
        """ Boucle de contrôle de l'action """
        target = goal_handle.request.pose.pose.position
        self.get_logger().info(f'Nouveau but reçu: x={target.x:.2f}, y={target.y:.2f}')

        # Conversion ENU (Nav2) -> NED (PX4)
        # Note: Dans la plupart des simus Gazebo/PX4 : NED_X = ENU_Y, NED_Y = ENU_X
        ned_x = float(target.y)
        ned_y = float(target.x)
        ned_z = float(-self.target_alt)

        self.active_setpoint = [ned_x, ned_y, ned_z]

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

            if self.current_pose:
                dist = math.sqrt((ned_x - self.current_pose.x)**2 + (ned_y - self.current_pose.y)**2)
                
                feedback = NavigateToPose.Feedback()
                feedback.distance_remaining = dist
                goal_handle.publish_feedback(feedback)

                if dist < self.tolerance:
                    self.get_logger().info(f"But atteint pour {self.ns}")
                    goal_handle.succeed()
                    return NavigateToPose.Result()

            # On attend un peu avant la prochaine itération
            import asyncio
            await asyncio.sleep(0.1)

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Action annulée')
        return CancelResponse.ACCEPT

    def send_cmd(self, command, p1, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Nav2ActionBridge())
    rclpy.shutdown()
