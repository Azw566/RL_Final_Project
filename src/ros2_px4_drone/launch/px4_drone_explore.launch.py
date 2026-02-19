from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    px4_drone_dir = FindPackageShare('ros2_px4_drone')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    explore_params_file = LaunchConfiguration('explore_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    startup_target_x = LaunchConfiguration('startup_target_x')
    startup_target_y = LaunchConfiguration('startup_target_y')
    startup_tolerance = LaunchConfiguration('startup_tolerance')
    startup_kp = LaunchConfiguration('startup_kp')
    startup_target_in_world = LaunchConfiguration('startup_target_in_world')
    startup_spawn_world_x = LaunchConfiguration('startup_spawn_world_x')
    startup_spawn_world_y = LaunchConfiguration('startup_spawn_world_y')
    startup_min_altitude = LaunchConfiguration('startup_min_altitude')

    # 1. SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([px4_drone_dir, 'launch', 'px4_drone_slam.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 2. NAV2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    # 3. Explore Lite
    explore_lite_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    # 4. TFs
    rplidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rplidar_static_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'rplidar_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nav2_px4_bridge = Node(
        package='ros2_px4_drone',
        executable='nav2_px4_bridge',
        name='nav2_px4_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'px4_namespace': namespace,
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'startup_reposition': True,
            'startup_target_x': startup_target_x,
            'startup_target_y': startup_target_y,
            'startup_tolerance': startup_tolerance,
            'startup_kp': startup_kp,
            'startup_target_in_world': startup_target_in_world,
            'startup_spawn_world_x': startup_spawn_world_x,
            'startup_spawn_world_y': startup_spawn_world_y,
            'startup_min_altitude': startup_min_altitude,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='drone1'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([px4_drone_dir, 'config', 'navigation.yaml'])),
        DeclareLaunchArgument('explore_params_file', default_value=PathJoinSubstitution([px4_drone_dir, 'config', 'explore.yaml'])),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([px4_drone_dir, 'rviz_conf', 'explore.rviz'])),
        DeclareLaunchArgument('startup_target_x', default_value='0.0'),
        DeclareLaunchArgument('startup_target_y', default_value='0.0'),
        DeclareLaunchArgument('startup_tolerance', default_value='2.5'),
        DeclareLaunchArgument('startup_kp', default_value='0.6'),
        DeclareLaunchArgument('startup_target_in_world', default_value='true'),
        DeclareLaunchArgument('startup_spawn_world_x', default_value='0.0'),
        DeclareLaunchArgument('startup_spawn_world_y', default_value='-8.0'),
        DeclareLaunchArgument('startup_min_altitude', default_value='0.8'),
        base_link_tf,
        rplidar_tf,
        slam_launch,
        nav2_bringup_launch,
        TimerAction(period=35.0, actions=[explore_lite_node]),
        nav2_px4_bridge,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
