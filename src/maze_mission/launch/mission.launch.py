"""
mission.launch.py — Launches only the mission_planner node.

The mission_planner subscribes to /map and /aruco/detections and
coordinates the overall mission state machine.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    planner = Node(
        package='maze_mission',
        executable='mission_planner',
        name='mission_planner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'exploration_coverage_threshold': 0.60,
            'max_tags': 5,
            'drone_namespaces': ['drone1', 'drone2'],
        }],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        planner,
    ])
