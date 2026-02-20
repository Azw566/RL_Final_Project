"""
turtlebot.launch.py — Launches the full TurtleBot 4 integration stack.

Includes:
  - nav2_bringup navigation_launch.py (bt_navigator, controller, planner, recoveries)
    NOTE: navigation_launch does NOT include map_server — that is intentional.
  - nav2_amcl     (standalone lifecycle node — localizes against /map from map_merge)
  - lifecycle_manager_localization (manages AMCL lifecycle)
  - origin_sync_node (publishes /initialpose to bootstrap AMCL)
  - turtlebot_controller (goal dispatcher from mission_planner)
  - aruco_detector (TurtleBot front camera)

The TurtleBot does NOT use map_server — the map comes from
multirobot_map_merge on /map.  nav2_bringup's bringup_launch.py is
intentionally NOT used because it forces map_server (which would conflict
with or pre-empt the map_merge output on /map).

Arguments:
  tb_x, tb_y, tb_yaw — TurtleBot's spawn pose in the Gazebo/map frame
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maze_mission_dir = get_package_share_directory('maze_mission')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(maze_mission_dir, 'config', 'turtlebot_nav2_params.yaml')
    aruco_config = os.path.join(maze_mission_dir, 'config', 'aruco_params.yaml')

    # ── Arguments ───────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    tb_x_arg = DeclareLaunchArgument('tb_x', default_value='0.0')
    tb_y_arg = DeclareLaunchArgument('tb_y', default_value='0.0')
    tb_yaw_arg = DeclareLaunchArgument('tb_yaw', default_value='0.0')

    use_sim_time = LaunchConfiguration('use_sim_time')
    tb_x = LaunchConfiguration('tb_x')
    tb_y = LaunchConfiguration('tb_y')
    tb_yaw = LaunchConfiguration('tb_yaw')

    # ── Nav2 navigation stack (bt_navigator, controller, planner, recoveries) ─
    # navigation_launch.py does NOT include map_server — exactly what we need.
    # The map comes live from /map published by multirobot_map_merge.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'use_composition': 'False',
        }.items(),
    )

    # ── AMCL — localizes TurtleBot in the drone's merged map ─────────────────
    # Runs as a standalone lifecycle node (managed by lifecycle_manager_localization
    # below) so we avoid bringup_launch.py which would pull in map_server.
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    # ── Lifecycle manager for AMCL ───────────────────────────────────────────
    # Configures and activates AMCL independently from the navigation stack.
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl'],
        }],
    )

    # ── origin_sync_node ────────────────────────────────────────────────────
    origin_sync = Node(
        package='maze_mission',
        executable='origin_sync_node',
        name='origin_sync_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'initial_x': tb_x,
            'initial_y': tb_y,
            'initial_yaw': tb_yaw,
            'publish_count': 5,
            'publish_interval': 1.0,
        }],
        output='screen',
    )

    # ── turtlebot_controller ─────────────────────────────────────────────────
    controller = Node(
        package='maze_mission',
        executable='turtlebot_controller',
        name='turtlebot_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── aruco_detector (TurtleBot front camera) ──────────────────────────────
    aruco_det = Node(
        package='maze_mission',
        executable='aruco_detector',
        name='aruco_detector',
        namespace='turtlebot',
        parameters=[aruco_config, {
            'use_sim_time': use_sim_time,
            'camera_ns': 'turtlebot',
            'camera_frame': 'oakd_rgb_camera_optical_frame',
        }],
        remappings=[
            ('image_raw', '/turtlebot/oakd/rgb/image_raw'),
            ('camera_info', '/turtlebot/oakd/rgb/camera_info'),
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        tb_x_arg,
        tb_y_arg,
        tb_yaw_arg,
        navigation,
        amcl,
        lifecycle_manager_localization,
        origin_sync,
        controller,
        aruco_det,
    ])
