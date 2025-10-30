cmake_minimum_required(VERSION 3.10)
project(teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(_python_build_dir "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}")
if(EXISTS "${_python_build_dir}")
  file(REMOVE_RECURSE "${_python_build_dir}")
endif()

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    teleop/teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

add_executable(local_planner_node src/local_planner_node.cpp)

ament_target_dependencies(
  local_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  octomap
  octomap_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

install(
  TARGETS local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
