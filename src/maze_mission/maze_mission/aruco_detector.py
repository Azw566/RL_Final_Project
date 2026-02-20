#!/usr/bin/env python3
"""
aruco_detector.py — Detects ArUco markers in camera images and publishes
their 3D poses in the map frame.

Subscribes:
    ~image_raw   (sensor_msgs/Image)
    ~camera_info (sensor_msgs/CameraInfo)

Publishes:
    /aruco/detections (maze_msgs/ArucoDetectionArray)

Parameters:
    marker_size   — physical ArUco marker side length in meters (default 0.15)
    aruco_dict    — ArUco dictionary name (default "DICT_4X4_50")
    camera_ns     — namespace identifying which camera/robot this is
    map_frame     — the global map frame (default "map")
"""

import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import tf2_ros
import tf2_geometry_msgs

from maze_msgs.msg import ArucoDetection, ArucoDetectionArray


ARUCO_DICT_MAP = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
}


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('camera_ns', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_optical_link')

        self.marker_size = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('aruco_dict').value
        self.camera_ns = self.get_parameter('camera_ns').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        aruco_dict_id = ARUCO_DICT_MAP.get(dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # TF2 buffer for map transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.img_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, 'camera_info', self.info_callback, 10
        )

        # Publisher
        self.detection_pub = self.create_publisher(
            ArucoDetectionArray, '/aruco/detections', 10
        )

        self.get_logger().info(
            f'ArucoDetector ready (dict={dict_name}, size={self.marker_size}m, '
            f'camera_ns={self.camera_ns})'
        )

    def info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return

        detection_array = ArucoDetectionArray()
        detection_array.header.stamp = msg.header.stamp
        detection_array.header.frame_id = self.map_frame

        # Half-size marker in 3D — object points for solvePnP
        half = self.marker_size / 2.0
        obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        for i, corner in enumerate(corners):
            marker_id = int(ids[i][0])
            img_points = corner[0].astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_points,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not success:
                continue

            # Build pose in camera frame
            pose_cam = PoseStamped()
            pose_cam.header.stamp = msg.header.stamp
            pose_cam.header.frame_id = self.camera_frame
            pose_cam.pose.position.x = float(tvec[0])
            pose_cam.pose.position.y = float(tvec[1])
            pose_cam.pose.position.z = float(tvec[2])

            # Convert rvec to quaternion via rotation matrix
            rmat, _ = cv2.Rodrigues(rvec)
            quat = self._rotation_matrix_to_quaternion(rmat)
            pose_cam.pose.orientation.x = quat[0]
            pose_cam.pose.orientation.y = quat[1]
            pose_cam.pose.orientation.z = quat[2]
            pose_cam.pose.orientation.w = quat[3]

            # Transform to map frame
            try:
                pose_map = self.tf_buffer.transform(
                    pose_cam, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except Exception as e:
                self.get_logger().debug(f'TF transform failed for marker {marker_id}: {e}')
                continue

            detection = ArucoDetection()
            detection.header = pose_map.header
            detection.marker_id = marker_id
            detection.pose = pose_map
            detection.confidence = 1.0
            detection.camera_namespace = self.camera_ns

            detection_array.detections.append(detection)
            self.get_logger().info(
                f'Detected ArUco ID {marker_id} at map '
                f'({pose_map.pose.position.x:.2f}, {pose_map.pose.position.y:.2f})'
            )

        if detection_array.detections:
            self.detection_pub.publish(detection_array)

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """Convert 3x3 rotation matrix to (x, y, z, w) quaternion."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        # Ensure positive w
        if w < 0:
            x, y, z, w = -x, -y, -z, -w
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        return (x/norm, y/norm, z/norm, w/norm)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
