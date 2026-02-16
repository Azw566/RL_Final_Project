import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    
    # --- PATHS ---
    pkg_ros2_fra2mo = get_package_share_directory('ros2_fra2mo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_robotic_bar = get_package_share_directory('robotic_bar_package')
    map_file = os.path.join(pkg_ros2_fra2mo, 'maps', 'map.yaml')
    nav_params_file = os.path.join(pkg_ros2_fra2mo, 'config', 'navigation.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ====================================================
    # NAVIGATION
    # ====================================================
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/fra2mo/cmd_vel'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': nav_params_file,
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                    'use_composition': 'True',
                }.items(),
            )
        ]
    )
    # ====================================================
    # VISION
    # ====================================================
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'marker_id': 0,
            'marker_size': 0.3,
            'reference_frame': 'base_footprint',
            'camera_frame': 'camera_link_optical',
            'marker_frame': 'aruco_marker_frame',
            'image_is_rectified': True,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/camera_info', '/stereo/left/camera_info'),
            ('/image', '/stereo/left/image_rect_color')
        ],
        output='screen'
    )

    rqt_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/aruco_single/result'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ====================================================
    # BRAINS 
    # ====================================================
    kuka_brain = Node(
        package='robotic_bar_package',
        executable='kuka_brain',
        name='kuka_brain',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    fra2mo_brain = Node(
        package='robotic_bar_package',
        executable='fra2mo_brain',
        name='fra2mo_brain',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # EVENT HANDLER: Activate BRAINS when ARUCO node starts
    brains_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=aruco_node,
            on_start=[kuka_brain, fra2mo_brain]
        )
    )

    return LaunchDescription([
        nav2_launch,
        aruco_node,
        rqt_view,
        brains_handler
    ])