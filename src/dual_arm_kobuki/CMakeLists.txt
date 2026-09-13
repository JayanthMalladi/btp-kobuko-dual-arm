cmake_minimum_required(VERSION 3.8)
project(dual_arm_kobuki)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    nodes/pose_cmd.py
    nodes/arm_kinematics.py
    nodes/pick_and_place.py
    nodes/coupling_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
