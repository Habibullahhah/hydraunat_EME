cmake_minimum_required(VERSION 3.8)
project(loco_gazebo)

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

# --- FIX: TELL COMPILER WHERE HEADERS ARE ---
include_directories(
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

# 2. Build the Hydrodynamics Plugin
add_library(simple_hydrodynamics SHARED src/simple_hydrodynamics.cpp)
target_link_libraries(simple_hydrodynamics 
  ${gazebo_dev_LIBRARIES} 
  ${gazebo_ros_LIBRARIES}
  rclcpp::rclcpp
)

# 3. Install the Plugin
install(TARGETS simple_hydrodynamics
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Install the Python Node
install(PROGRAMS
  src/sim_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Install Launch files and Worlds
install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
