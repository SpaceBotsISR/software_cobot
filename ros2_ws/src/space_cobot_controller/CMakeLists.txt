cmake_minimum_required(VERSION 3.8)
project(space_cobot_controller)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(tf2 REQUIRED)
find_package(space_cobot_interface REQUIRED)
find_package(mocap_interface REQUIRED)
find_package(mav_msgs REQUIRED)

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
  
set (executable_name 
  spaceCobotController
)
  add_executable(${executable_name} 
  src/space_cobot_controller.cpp 
  src/AttitideControl.h 
  src/AMatrix.h 
  src/PositionControl.h 
  src/Actuation.h 
  src/AttitudeControl.cpp 
  src/PositionControl.cpp 
  src/AMatrix.cpp 
  src/Actuation.cpp
)

ament_target_dependencies ( ${executable_name} 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  mavros_msgs 
  std_msgs 
  tf2 
  mocap_interface 
  mav_msgs 
  space_cobot_interface
)

target_link_libraries(spaceCobotController ${rclcpp_LIBRARIES}  ${OpenCV_LIBS})
  
ament_export_dependencies(rosidl_default_runtime
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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
