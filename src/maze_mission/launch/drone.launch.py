"""
drone.launch.py — Launches all nodes for a single drone.

Includes:
  - ros_gz_bridge   (clock + LiDAR scan)
  - odom_converter  (from PX4-ROS2-SLAM-Control, NED→ENU)
  - slam_toolbox    (mapping)
  - nav2_costmap_2d (for explore_lite)
  - explore_lite    (frontier exploration)
  - nav2_action_bridge (fake Nav2 server for the drone)
  - aruco_detector  (camera → map-frame detections)

Arguments:
  namespace   — drone ROS 2 namespace (e.g. "drone1")
  gz_model    — Gazebo model name for the LiDAR bridge (e.g. "x500_lidar_2d_0")
  init_x/y    — initial drone position in ENU for SLAM origin (meters)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    maze_mission_dir = get_package_share_directory('maze_mission')

    # ── Launch arguments ────────────────────────────────────────────────────
    ns_arg = DeclareLaunchArgument('namespace', default_value='drone1')
    gz_model_arg = DeclareLaunchArgument('gz_model', default_value='x500_lidar_2d_0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ns = LaunchConfiguration('namespace')
    gz_model = LaunchConfiguration('gz_model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_config = os.path.join(maze_mission_dir, 'config', 'drone_slam_params.yaml')
    explore_config = os.path.join(maze_mission_dir, 'config', 'drone_explore.yaml')
    costmap_config = os.path.join(maze_mission_dir, 'config', 'drone_costmap.yaml')
    aruco_config = os.path.join(maze_mission_dir, 'config', 'aruco_params.yaml')

    # ── Nodes ───────────────────────────────────────────────────────────────

    # 1. ros_gz_bridge: forward Gazebo clock + LiDAR scan to ROS 2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=ns,
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # LiDAR scan: Gazebo topic → ROS 2 topic (remapped via namespace)
            ['/model/', gz_model, '/link/lidar_link/sensor/lidar/scan'
             '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        ],
        remappings=[
            (['/model/', gz_model, '/link/lidar_link/sensor/lidar/scan'], 'scan'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 2. odom_converter (from PX4-ROS2-SLAM-Control)
    odom_converter = Node(
        package='drone_slam',
        executable='odom_converter',
        namespace=ns,
        name='odom_converter',
        remappings=[
            ('/fmu/out/vehicle_odometry', ['/', ns, '/fmu/out/vehicle_odometry']),
            ('/odom', ['/', ns, '/odom']),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 3. Static TF: map → odom (initial, overridden by slam_toolbox)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=ns,
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 4. slam_toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace=ns,
        name='slam_toolbox',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/map', ['/', ns, '/map']),
            ('/map_metadata', ['/', ns, '/map_metadata']),
        ],
        output='screen',
    )

    # 5. costmap_2d (global, for explore_lite)
    costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace=ns,
        name='global_costmap',
        parameters=[costmap_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
    )

    # 6. explore_lite (frontier exploration)
    # In m-explore-ros2 Humble, ACTION_NAME is already 'navigate_to_pose' —
    # no remapping needed. The action client resolves to /<ns>/navigate_to_pose
    # which is served by nav2_action_bridge below.
    explore = Node(
        package='explore_lite',
        executable='explore',
        namespace=ns,
        name='explore_node',
        parameters=[explore_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 7. nav2_action_bridge (fake NavigateToPose action server for this drone)
    nav2_bridge = Node(
        package='maze_mission',
        executable='nav2_action_bridge',
        namespace=ns,
        name='nav2_action_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'altitude': 1.2,
            'threshold': 0.5,
            'ns': ns,
        }],
        output='screen',
    )

    # 8. aruco_detector (drone camera)
    aruco_det = Node(
        package='maze_mission',
        executable='aruco_detector',
        namespace=ns,
        name='aruco_detector',
        parameters=[aruco_config, {
            'use_sim_time': use_sim_time,
            'camera_ns': ns,
            'camera_frame': [ns, '/camera_optical_link'],
        }],
        remappings=[
            ('image_raw', 'camera/image_raw'),
            ('camera_info', 'camera/camera_info'),
        ],
        output='screen',
    )

    return LaunchDescription([
        ns_arg,
        gz_model_arg,
        use_sim_time_arg,
        gz_bridge,
        odom_converter,
        static_tf_map_odom,
        slam_toolbox,
        costmap,
        explore,
        nav2_bridge,
        aruco_det,
    ])
