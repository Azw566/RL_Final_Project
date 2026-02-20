"""
gazebo.launch.py — Launches Gazebo Harmonic with the maze world.

Also spawns the TurtleBot 4 model if turtlebot4_simulator is available.
PX4 SITL drones are spawned externally via scripts/launch_px4_sitl.sh.

Sets GZ_SIM_RESOURCE_PATH so Gazebo can resolve model://aruco_tags/tag*.png
texture references used in maze.sdf.

Arguments:
  world     — path to SDF world (default: maze.sdf)
  tb_x/y/yaw — TurtleBot spawn pose
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maze_gazebo_dir = get_package_share_directory('maze_gazebo')
    world_path = os.path.join(maze_gazebo_dir, 'worlds', 'maze.sdf')
    models_dir = os.path.join(maze_gazebo_dir, 'models')

    # Append our models directory to the existing GZ_SIM_RESOURCE_PATH so that
    # model://aruco_tags/tag*.png textures referenced in maze.sdf are found.
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f'{models_dir}:{existing_gz_path}' if existing_gz_path else models_dir

    world_arg = DeclareLaunchArgument('world', default_value=world_path)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    tb_x_arg = DeclareLaunchArgument('tb_x', default_value='1.0')
    tb_y_arg = DeclareLaunchArgument('tb_y', default_value='1.0')
    tb_yaw_arg = DeclareLaunchArgument('tb_yaw', default_value='0.0')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    tb_x = LaunchConfiguration('tb_x')
    tb_y = LaunchConfiguration('tb_y')
    tb_yaw = LaunchConfiguration('tb_yaw')

    # Launch Gazebo Harmonic with the resource path set
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen',
    )

    # Spawn TurtleBot 4 (requires turtlebot4_simulator)
    spawn_turtlebot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'maze',
            '-name', 'turtlebot4',
            '-topic', '/robot_description',
            '-x', tb_x,
            '-y', tb_y,
            '-z', '0.01',
            '-Y', tb_yaw,
        ],
        output='screen',
    )

    # Robot state publisher for TurtleBot 4 URDF
    try:
        tb4_desc_dir = get_package_share_directory('turtlebot4_description')
        tb4_urdf = os.path.join(tb4_desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro')
        robot_state_pub = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                '--ros-args',
                '-p', f'robot_description:=$(xacro {tb4_urdf})',
                '-p', 'use_sim_time:=true',
            ],
            output='screen',
        )
        tb4_nodes = [robot_state_pub, spawn_turtlebot]
    except Exception:
        # turtlebot4_description not installed — skip TurtleBot spawn
        tb4_nodes = []

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        tb_x_arg,
        tb_y_arg,
        tb_yaw_arg,
        gz_sim,
        *tb4_nodes,
    ])
