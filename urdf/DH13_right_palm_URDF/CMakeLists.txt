cmake_minimum_required(VERSION 3.8)
project(DH13_right_palm_URDF)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
