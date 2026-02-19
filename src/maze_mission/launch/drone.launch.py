from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    enable_aruco = LaunchConfiguration('enable_aruco')
    aruco_image_topic = LaunchConfiguration('aruco_image_topic')
    aruco_camera_info_topic = LaunchConfiguration('aruco_camera_info_topic')
    aruco_marker_size = LaunchConfiguration('aruco_marker_size')
    aruco_use_camera_info = LaunchConfiguration('aruco_use_camera_info')
    aruco_camera_frame = LaunchConfiguration('aruco_camera_frame')
    aruco_reference_frame = LaunchConfiguration('aruco_reference_frame')
    startup_target_x = LaunchConfiguration('startup_target_x')
    startup_target_y = LaunchConfiguration('startup_target_y')
    startup_tolerance = LaunchConfiguration('startup_tolerance')
    startup_kp = LaunchConfiguration('startup_kp')
    startup_target_in_world = LaunchConfiguration('startup_target_in_world')
    startup_spawn_world_x = LaunchConfiguration('startup_spawn_world_x')
    startup_spawn_world_y = LaunchConfiguration('startup_spawn_world_y')
    startup_min_altitude = LaunchConfiguration('startup_min_altitude')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='drone1', description='Namespace for the drone'
    )

    declare_enable_aruco = DeclareLaunchArgument(
        'enable_aruco', default_value='true', description='Run ArUco marker publisher node'
    )

    declare_aruco_image_topic = DeclareLaunchArgument(
        'aruco_image_topic', default_value='/camera/image_raw', description='Image topic for ArUco detection'
    )

    declare_aruco_camera_info_topic = DeclareLaunchArgument(
        'aruco_camera_info_topic', default_value='/camera/camera_info', description='CameraInfo topic for ArUco detection'
    )

    declare_aruco_marker_size = DeclareLaunchArgument(
        'aruco_marker_size', default_value='0.20', description='ArUco marker size in meters'
    )

    declare_aruco_use_camera_info = DeclareLaunchArgument(
        'aruco_use_camera_info', default_value='false', description='Require camera_info for 3D marker pose'
    )

    declare_aruco_camera_frame = DeclareLaunchArgument(
        'aruco_camera_frame', default_value='base_link', description='Camera frame id used by ArUco node'
    )

    declare_aruco_reference_frame = DeclareLaunchArgument(
        'aruco_reference_frame', default_value='map', description='Reference frame for marker pose output'
    )

    declare_startup_target_x = DeclareLaunchArgument(
        'startup_target_x', default_value='0.0', description='Initial push target X in odom frame'
    )

    declare_startup_target_y = DeclareLaunchArgument(
        'startup_target_y', default_value='0.0', description='Initial push target Y (world or odom frame)'
    )

    declare_startup_tolerance = DeclareLaunchArgument(
        'startup_tolerance', default_value='2.5', description='Distance threshold to stop initial push'
    )

    declare_startup_kp = DeclareLaunchArgument(
        'startup_kp', default_value='0.6', description='Proportional gain for initial push'
    )

    declare_startup_target_in_world = DeclareLaunchArgument(
        'startup_target_in_world', default_value='true', description='Interpret startup target in Gazebo world frame'
    )

    declare_startup_spawn_world_x = DeclareLaunchArgument(
        'startup_spawn_world_x', default_value='0.0', description='Expected spawn X in Gazebo world frame'
    )

    declare_startup_spawn_world_y = DeclareLaunchArgument(
        'startup_spawn_world_y', default_value='-8.0', description='Expected spawn Y in Gazebo world frame'
    )

    declare_startup_min_altitude = DeclareLaunchArgument(
        'startup_min_altitude', default_value='0.8', description='Minimum altitude before startup translation'
    )

    bt_xml_path = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'

    # Include the full exploration stack
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros2_px4_drone'), 'launch', 'px4_drone_explore.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            # On force le paramètre ici pour écraser le YAML
            'default_bt_xml_filename': bt_xml_path,
            'startup_target_x': startup_target_x,
            'startup_target_y': startup_target_y,
            'startup_tolerance': startup_tolerance,
            'startup_kp': startup_kp,
            'startup_target_in_world': startup_target_in_world,
            'startup_spawn_world_x': startup_spawn_world_x,
            'startup_spawn_world_y': startup_spawn_world_y,
            'startup_min_altitude': startup_min_altitude,
        }.items(),
    )

    mission_planner_node = Node(
        package='maze_mission',
        executable='mission_planner',
        name='mission_planner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'namespace': namespace 
        }],
        output='screen',
    )

    aruco_node = Node(
        package='aruco_ros',
        executable='marker_publisher',
        namespace='aruco',
        name='marker_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'marker_size': aruco_marker_size,
            'camera_frame': aruco_camera_frame,
            'reference_frame': aruco_reference_frame,
            'image_is_rectified': True,
            'use_camera_info': aruco_use_camera_info,
        }],
        remappings=[
            ('/image', aruco_image_topic),
            ('/camera_info', aruco_camera_info_topic),
        ],
        condition=IfCondition(enable_aruco),
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        declare_enable_aruco,
        declare_aruco_image_topic,
        declare_aruco_camera_info_topic,
        declare_aruco_marker_size,
        declare_aruco_use_camera_info,
        declare_aruco_camera_frame,
        declare_aruco_reference_frame,
        declare_startup_target_x,
        declare_startup_target_y,
        declare_startup_tolerance,
        declare_startup_kp,
        declare_startup_target_in_world,
        declare_startup_spawn_world_x,
        declare_startup_spawn_world_y,
        declare_startup_min_altitude,
        explore_launch,
        aruco_node,
        mission_planner_node,
    ])
