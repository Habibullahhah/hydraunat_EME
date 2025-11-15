cmake_minimum_required(VERSION 3.8)
project(loco_pilot)

# 1. Find the tools we need
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. Generate the Message Code
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Command.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# 3. Register the package
ament_package()
