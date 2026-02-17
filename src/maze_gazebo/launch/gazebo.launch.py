import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # 1. Locate the package paths
    pkg_name = 'maze_gazebo'
    pkg_share = get_package_share_directory(pkg_name)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # 2. Setup the world argument (Now defaulting to rescue_maze.world)
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='rescue_maze.world',
        description='Name of the world file in the worlds/ folder'
    )

    # 3. Use PathJoinSubstitution for cleaner path handling
    world_path = PathJoinSubstitution([
        pkg_share,
        'worlds',
        LaunchConfiguration('world')
    ])

    # 4. Include the Gazebo Server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 5. Include the Gazebo Client (The UI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        )
    )

    return LaunchDescription([
        world_file_arg,
        gzserver,
        gzclient
    ])
