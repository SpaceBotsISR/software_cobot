cmake_minimum_required(VERSION 3.10)
project(teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)

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

add_executable(teleop_node src/teleop_bridge_node.cpp)

ament_target_dependencies(
  teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(
  teleop_node
  PkgConfig::ZMQ
  ${OpenCV_LIBRARIES}
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

target_include_directories(
  teleop_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

install(
  TARGETS
    teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
