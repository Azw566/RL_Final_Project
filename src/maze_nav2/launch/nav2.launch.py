"""
nav2.launch.py — Standalone Nav2 launch for the TurtleBot 4.

Uses maze_nav2 params. Does NOT use map_server — the map comes live from
/map published by multirobot_map_merge.

Can be included from turtlebot.launch.py or run independently for development.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maze_nav2_dir = get_package_share_directory('maze_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(maze_nav2_dir, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Navigation stack (bt_navigator, controller, planner, recoveries) — no map_server
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_composition': 'False',
        }.items(),
    )

    # AMCL — standalone lifecycle node localizing against /map from map_merge
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # Lifecycle manager for AMCL
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

    return LaunchDescription([
        use_sim_time_arg,
        navigation,
        amcl,
        lifecycle_manager_localization,
    ])
