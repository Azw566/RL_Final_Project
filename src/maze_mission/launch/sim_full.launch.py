"""
sim_full.launch.py — Master launch file that brings up the entire simulation.

Launch order (as per architecture doc Section 10):
  1. Gazebo world (maze_gazebo)
  2. Map merge node   (must be before explore_lite)
  3. Both drones      (gz_bridge + odom_converter + slam + costmap + explore + nav2_bridge + aruco)
  4. TurtleBot        (Nav2 + AMCL + origin_sync + controller + aruco)
  5. Mission planner  (monitoring + dispatching)

NOTE: PX4 SITL instances and XRCE-DDS agents are launched separately via
      scripts/launch_px4_sitl.sh because they are external processes.

Arguments:
  use_sim_time — default true
  tb_x, tb_y, tb_yaw — TurtleBot spawn pose in map frame
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    maze_mission_dir = get_package_share_directory('maze_mission')
    maze_gazebo_dir = get_package_share_directory('maze_gazebo')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    tb_x_arg = DeclareLaunchArgument('tb_x', default_value='1.0')
    tb_y_arg = DeclareLaunchArgument('tb_y', default_value='1.0')
    tb_yaw_arg = DeclareLaunchArgument('tb_yaw', default_value='0.0')

    use_sim_time = LaunchConfiguration('use_sim_time')
    tb_x = LaunchConfiguration('tb_x')
    tb_y = LaunchConfiguration('tb_y')
    tb_yaw = LaunchConfiguration('tb_yaw')

    # ── 1. Gazebo ────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(maze_gazebo_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── 2. Map merge (start early so costmaps can subscribe on boot) ─────────
    map_merge = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(maze_mission_dir, 'launch', 'map_merge.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    # ── 3. Both drones ───────────────────────────────────────────────────────
    drones = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(maze_mission_dir, 'launch', 'drones.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    # ── 4. TurtleBot ─────────────────────────────────────────────────────────
    turtlebot = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(maze_mission_dir, 'launch', 'turtlebot.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'tb_x': tb_x,
                'tb_y': tb_y,
                'tb_yaw': tb_yaw,
            }.items(),
        )],
    )

    # ── 5. Mission planner ───────────────────────────────────────────────────
    mission = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(maze_mission_dir, 'launch', 'mission.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    return LaunchDescription([
        use_sim_time_arg,
        tb_x_arg,
        tb_y_arg,
        tb_yaw_arg,
        gazebo,
        map_merge,
        drones,
        turtlebot,
        mission,
    ])
