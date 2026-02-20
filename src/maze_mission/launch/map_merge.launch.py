"""
map_merge.launch.py — Launches multirobot_map_merge.

Subscribes to /drone1/map and /drone2/map (both ENU OccupancyGrids from
slam_toolbox). Publishes the merged map on /map.

Must be launched BEFORE explore_lite so that the per-drone costmaps
have a map to reference on startup.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maze_mission_dir = get_package_share_directory('maze_mission')
    map_merge_config = os.path.join(maze_mission_dir, 'config', 'map_merge_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_merge_node = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        parameters=[map_merge_config, {'use_sim_time': use_sim_time}],
        remappings=[
            # The node auto-discovers namespaced /droneN/map topics
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_merge_node,
    ])
