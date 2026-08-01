cmake_minimum_required(VERSION 3.8)
project(sstg_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES UPSTREAM.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
