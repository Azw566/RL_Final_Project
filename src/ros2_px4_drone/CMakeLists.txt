cmake_minimum_required(VERSION 3.5)
project(ros2_px4_drone)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(nav2_px4_bridge src/nav2_px4_bridge.cpp)
ament_target_dependencies(nav2_px4_bridge rclcpp geometry_msgs nav_msgs px4_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(px4_drone_controller src/px4_drone_controller.cpp)
ament_target_dependencies(px4_drone_controller rclcpp geometry_msgs px4_msgs tf2 tf2_geometry_msgs)

install(DIRECTORY launch config rviz_conf scripts
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS nav2_px4_bridge px4_drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
