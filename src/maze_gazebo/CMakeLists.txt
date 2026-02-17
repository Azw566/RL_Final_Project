cmake_minimum_required(VERSION 3.8)
project(maze_gazebo)

find_package(ament_cmake REQUIRED)

# Install models and worlds
install(DIRECTORY
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
