cmake_minimum_required(VERSION 3.5)
project(test_urdf)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find ament_cmake package
find_package(ament_cmake REQUIRED)

# Install the URDF and meshes directories
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Finalize the package
ament_package()
