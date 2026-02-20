"""
drones.launch.py — Launches the full stack for both drones.

Each drone runs drone.launch.py with a different namespace and Gazebo model name.

Drone 1: namespace=drone1, gz_model=x500_lidar_2d_0
Drone 2: namespace=drone2, gz_model=x500_lidar_2d_1
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    maze_mission_dir = get_package_share_directory('maze_mission')
    drone_launch = os.path.join(maze_mission_dir, 'launch', 'drone.launch.py')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    drone1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drone_launch),
        launch_arguments={
            'namespace': 'drone1',
            'gz_model': 'x500_lidar_2d_0',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    drone2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drone_launch),
        launch_arguments={
            'namespace': 'drone2',
            'gz_model': 'x500_lidar_2d_1',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        drone1,
        drone2,
    ])
