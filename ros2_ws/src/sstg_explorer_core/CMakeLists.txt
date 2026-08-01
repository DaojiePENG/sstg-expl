cmake_minimum_required(VERSION 3.8)
project(sstg_explorer_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the existing, simulator-independent Python package into the ROS
# workspace.  Keeping one source tree prevents the procedural benchmark and
# ROS policy node from silently diverging.
ament_python_install_package(
  sstg_explorer
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/sstg_explorer"
)

ament_package()
