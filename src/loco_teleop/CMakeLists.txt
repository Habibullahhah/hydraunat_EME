cmake_minimum_required(VERSION 3.8)
project(loco_teleop)

find_package(ament_cmake REQUIRED)

# This command copies the script to the install folder
install(PROGRAMS
  src/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
