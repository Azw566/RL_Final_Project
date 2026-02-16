cmake_minimum_required(VERSION 3.8)
project(robotic_bar_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED) 
find_package(rclcpp_action REQUIRED) 
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

add_executable(kuka_brain src/kuka_brain.cpp)
ament_target_dependencies(kuka_brain rclcpp std_msgs trajectory_msgs) 

add_executable(fra2mo_brain src/fra2mo_brain.cpp)
ament_target_dependencies(fra2mo_brain
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)

install(TARGETS
  fra2mo_brain
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  kuka_brain
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS src/bar_button.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()