cmake_minimum_required(VERSION 3.8)
project(rescue_swarm_gazebo)

find_package(ament_cmake REQUIRED)

# Install models and worlds
install(DIRECTORY
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
