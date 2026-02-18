import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    px4_drone_dir = FindPackageShare('ros2_px4_drone')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    # 1. SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([px4_drone_dir, 'launch', 'px4_drone_slam.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 2. NAV2
    nav2_bringup_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst=['/', namespace, '/cmd_vel']),
            SetRemap(src='/odom', dst=['/', namespace, '/odom']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': 'True',
                }.items(),
            )
        ]
    )

    # 3. Explore Lite
    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('explore_lite'), 'launch', 'explore.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace}.items(),
    )
    
    # 4. TFs
    rplidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rplidar_static_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'rplidar_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # NOEUD CRITIQUE : Vérifie que son code interne utilise le timestamp simulé !
    dynamic_tf_publisher = Node(
        package='ros2_px4_drone',
        executable='dynamic_tf_publisher',
        name='dynamic_tf_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nav2_px4_bridge = Node(
        package='ros2_px4_drone',
        executable='nav2_px4_bridge',
        name='nav2_px4_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'cmd_vel_topic': ['/', namespace, '/cmd_vel'],
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='drone1'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([px4_drone_dir, 'config', 'navigation.yaml'])),
        rplidar_tf,
        dynamic_tf_publisher, 
        slam_launch,
        nav2_bringup_launch,
        explore_lite_launch,
        nav2_px4_bridge,
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': use_sim_time}])
    ])
