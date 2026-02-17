from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='drone1', description='Namespace for the drone'
    )

    # Include the full exploration stack (SLAM + Nav2 + explore_lite + bridge + RViz)
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros2_px4_drone'), 'launch', 'px4_drone_explore.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
        }.items(),
    )

    # Mission planner node
    mission_planner_node = Node(
        package='maze_mission',
        executable='mission_planner',
        name='mission_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        explore_launch,
        mission_planner_node,
    ])
