cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap_interface REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(space_cobot_interface REQUIRED)
find_package(OpenCV REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${space_cobot_interface_INCLUDE_DIRS}
  ${mocap_interface_INCLUDE_DIRS}
  ${mav_msgs_INCLUDE_DIRS}
)

include_directories(include)

set (control_master_name 
  control_master)

set(attitude_control_slave_name
  attitude_control_slave)

set (translation_control_slave_name
  translation_control_slave)

add_executable(${control_master_name} 
src/controller_master/controller_master_handle.cpp 
src/controller_master/controller_master_main.cpp
src/controller_master/Actuation.cpp
src/controller_master/AMatrix.cpp
)

add_executable(${attitude_control_slave_name}
src/attitude_controller/attitude_controller_main.cpp
src/attitude_controller/attitude_controller_handle.cpp
src/attitude_controller/euler_angles_input.cpp

)

ament_target_dependencies(${control_master_name}
  rclcpp
  geometry_msgs
  sensor_msgs
  mavros_msgs
  std_msgs
  tf2
  tf2_ros
  mocap_interface
  mav_msgs
  space_cobot_interface
)


ament_target_dependencies(${attitude_control_slave_name}
  rclcpp
  geometry_msgs
  sensor_msgs
  mavros_msgs
  std_msgs
  tf2
  tf2_ros
  mocap_interface
  mav_msgs
  space_cobot_interface
)

target_link_libraries(${control_master_name}
  ${colcon_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(${attitude_control_slave_name}
  ${colcon_LIBRARIES}
  ${OpenCV_LIBS}
)

install (
  TARGETS ${control_master_name}
  DESTINATION lib/${PROJECT_NAME}
)

install (
   TARGETS ${attitude_control_slave_name}
   DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
