cmake_minimum_required(VERSION 3.8)
project(loco_gazebo)

find_package(ament_cmake REQUIRED)

# Install the Python Node (Physics Logic)
install(PROGRAMS
  src/sim_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch files and Worlds
install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
