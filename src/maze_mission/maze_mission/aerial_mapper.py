#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleLocalPosition
from cv_bridge import CvBridge
import numpy as np
import cv2
import math

class AerialMapper(Node):
    def __init__(self):
        super().__init__('aerial_mapper')

        # Parameters
        self.declare_parameter('flight_altitude', 4.0)
        self.declare_parameter('wall_height_threshold', 2.0) # Anything higher than 2m is a wall
        self.declare_parameter('map_resolution', 0.1) # 10cm per pixel
        self.declare_parameter('map_size', 200) # 200x200 pixels (20x20m)

        self.flight_alt = self.get_parameter('flight_altitude').value
        self.wall_thresh = self.get_parameter('wall_height_threshold').value
        self.res = self.get_parameter('map_resolution').value
        self.map_size = self.get_parameter('map_size').value

        # QoS for PX4 (Best Effort is required for UDP bridge)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.bridge = CvBridge()
        self.create_subscription(Image, 'camera/depth/image_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, 'camera/depth/camera_info', self.info_cb, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pose_cb, qos_profile)

        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)

        # State
        self.current_pose = None
        self.camera_model = None
        self.grid = np.ones((self.map_size, self.map_size), dtype=np.int8) * -1 # Initialize unknown

        # Timer to publish map
        self.create_timer(1.0, self.publish_map)

    def info_cb(self, msg):
        self.camera_model = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def depth_cb(self, msg):
        if self.current_pose is None:
            return

        # Convert ROS Image to OpenCV
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # LOGIC:
        # Pixels with value ~ flight_altitude are FLOOR (Free)
        # Pixels with value < (flight_altitude - wall_height) are WALLS (Occupied)
        
        # Create masks
        # Note: Depth is in meters (float32)
        floor_mask = (depth_image > (self.flight_alt - 1.0)) # generous floor buffer
        wall_mask = (depth_image < (self.flight_alt - self.wall_thresh))

        # We need to map these pixels to the global grid. 
        # For simplicity in Phase 1, we assume the drone is mostly flat and camera points down.
        # We project the center of the image to the current (x,y) of the drone.
        
        # Drone position in map indices
        # Map center is (0,0) in world coordinates -> (size/2, size/2) in grid indices
        center_x = int(self.current_pose.x / self.res) + self.map_size // 2
        center_y = int(self.current_pose.y / self.res) + self.map_size // 2 # NED y is East.
        
        # Define a simple footprint size (e.g. 2x2 meters) around the drone to update
        # This is a simplification. A real projection requires tf2 and pinhole models.
        # For a top-down drone in a maze, this "Update area under drone" works well.
        
        range_px = int(2.0 / self.res) # 2 meter radius
        
        # Update grid (Simple slamming)
        # We look at the center pixel of the depth image to decide "Am I over a wall?"
        center_depth = depth_image[depth_image.shape[0]//2, depth_image.shape[1]//2]
        
        is_wall = center_depth < (self.flight_alt - self.wall_thresh)
        
        # Update the cells around the drone
        y_min = max(0, center_y - range_px)
        y_max = min(self.map_size, center_y + range_px)
        x_min = max(0, center_x - range_px)
        x_max = min(self.map_size, center_x + range_px)

        if is_wall:
            self.grid[y_min:y_max, x_min:x_max] = 100 # Occupied
        else:
            # Only mark free if it wasn't already marked as wall (simple persistence)
            # This logic prevents erasing walls seen previously
            current_slice = self.grid[y_min:y_max, x_min:x_max]
            # Set to 0 (Free) where it is currently -1 (Unknown) or 0 (Free)
            # Don't overwrite 100 (Occupied)
            mask_free = (current_slice != 100)
            self.grid[y_min:y_max, x_min:x_max][mask_free] = 0

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.res
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        
        # Origin is bottom-left. Since (0,0) world is center, origin is (-size/2, -size/2)
        origin = -(self.map_size * self.res) / 2.0
        msg.info.origin.position.x = origin
        msg.info.origin.position.y = origin
        msg.info.origin.position.z = 0.0
        
        # Flatten and convert to list
        msg.data = self.grid.flatten().tolist()
        
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AerialMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
