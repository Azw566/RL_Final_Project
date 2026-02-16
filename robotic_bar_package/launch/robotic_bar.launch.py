import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # --- PATHS ---
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_iiwa_description = get_package_share_directory("iiwa_description")
    pkg_robotic_bar_package = get_package_share_directory("robotic_bar_package")
    pkg_ros2_fra2mo = get_package_share_directory("ros2_fra2mo")


    models_path = os.path.join(pkg_robotic_bar_package, 'models')


    # ---  GAZEBO ---
    bar_sdf = os.path.join(pkg_robotic_bar_package, "worlds", "bar.sdf")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': ['-r ', '-v 4 ', bar_sdf]}.items(),
    )
    # ====================================================
    # IIWA ROBOT (Namespace: iiwa)
    # ====================================================
    xacro_iiwa = os.path.join(pkg_iiwa_description, "config", "iiwa.config.xacro")
    robot_desc_iiwa = ParameterValue(Command(["xacro ", xacro_iiwa]), value_type=str)

    rsp_iiwa = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="iiwa",
        output="screen",
        parameters=[{"robot_description": robot_desc_iiwa, "use_sim_time": True}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )

    spawn_iiwa = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/iiwa/robot_description",
            "-name", "iiwa",
            "-world", "robotic_bar_world",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
            "-ros_namespace", "iiwa"
        ],
        parameters=[{"use_sim_time": True}]
    )

    # ====================================================
    # FRA2MO ROBOT (Namespace: fra2mo)
    # ====================================================
    xacro_fra2mo = os.path.join(pkg_ros2_fra2mo, "urdf", "fra2mo.urdf.xacro")
    robot_desc_fra2mo = ParameterValue(Command(["xacro ", xacro_fra2mo]), value_type=str)

    rsp_fra2mo = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="fra2mo",
        output="screen",
        parameters=[{"robot_description": robot_desc_fra2mo, "use_sim_time": True}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )

    spawn_fra2mo = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/fra2mo/robot_description",
            "-name", "fra2mo",
            "-world", "robotic_bar_world",
            "-x", "-3.0", "-y", "6.0", "-z", "0.3",
            "-Y", "3.14",
            "-ros_namespace", "fra2mo"
        ],
        parameters=[{"use_sim_time": True}]
    )

    odom_tf_fra2mo = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static'), ('/model/fra2mo/odometry', '/fra2mo/odom')]
    )

    # ====================================================
    # Bridge
    # ====================================================
    bridges = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # --- CLOCK ---
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            
            # --- IIWA ---
            "/iiwa/bar/gripper/attach@std_msgs/msg/Empty@ignition.msgs.Empty",
            "/iiwa/bar/gripper/detach@std_msgs/msg/Empty@ignition.msgs.Empty",

            # --- FRA2MO ---
            "/fra2mo/bar/gripper/attach@std_msgs/msg/Empty@ignition.msgs.Empty",
            "/fra2mo/bar/gripper/detach@std_msgs/msg/Empty@ignition.msgs.Empty",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",

            # --- LIFT ---
            "/lift_1/elevation@std_msgs/msg/Float64@ignition.msgs.Double",
            "/lift_2/elevation@std_msgs/msg/Float64@ignition.msgs.Double",
            "/lift_3/elevation@std_msgs/msg/Float64@ignition.msgs.Double",
        ],
        remappings=[
            ('/cmd_vel', '/fra2mo/cmd_vel'),
            ('/model/fra2mo/odometry', '/fra2mo/odom'),
            ('/lidar', '/fra2mo/lidar'),
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
    )

    # --- Camera Bridge ---
    camera_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            # Topic Gazebo -> Tipo ROS -> Tipo Gazebo
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            
            # Remapping stile "ros-args"
            "--ros-args",
            "-r", "/camera:=/stereo/left/image_rect_color",
            "-r", "/camera_info:=/stereo/left/camera_info",
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
    )


    # ====================================================
    # 7. CONTROLLERS IIWA
    # ====================================================
    jsb_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa",
        arguments=["joint_state_broadcaster", "--controller-manager", "/iiwa/controller_manager"],
        parameters=[{'use_sim_time': True}]
    )

    traj_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa",
        arguments=["iiwa_arm_trajectory_controller", "--controller-manager", "/iiwa/controller_manager"],
        parameters=[{'use_sim_time': True}]
    )

    iiwa_controllers_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_iiwa,
            on_exit=[jsb_node, traj_node],
        )
    )

    
    # Spawn bridge
    creator_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="create_bridge",
        arguments=[
            "/world/robotic_bar_world/create@ros_gz_interfaces/srv/SpawnEntity"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # Deleter bridge
    remover_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="remove_bridge",
        arguments=[
            "/world/robotic_bar_world/remove@ros_gz_interfaces/srv/DeleteEntity"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription([
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH", 
            value = models_path + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ),
        
        gazebo_launch,
        
        rsp_iiwa, spawn_iiwa,
        rsp_fra2mo, spawn_fra2mo, odom_tf_fra2mo,
        bridges,
        camera_bridge,
        iiwa_controllers_handler,
        creator_bridge,
        remover_bridge
    ])