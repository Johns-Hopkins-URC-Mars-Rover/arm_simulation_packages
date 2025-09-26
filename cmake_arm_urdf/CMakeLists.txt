cmake_minimum_required(VERSION 3.5)
project(cmake_arm_urdf)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
