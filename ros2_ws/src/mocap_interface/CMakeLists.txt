cmake_minimum_required(VERSION 3.8)
project(mocap_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(mav_msgs REQUIRED)

# Include all package direcories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${tf_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${mav_msgs_INCLUDE_DIRS}
)

add_executable(mocapInterface 
  src/nodes/mocap_interface.cpp  
)

ament_target_dependencies(mocapInterface
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "mav_msgs")

set (msg_files
  "msg/MocapMsg.msg"
)

set (msg_file_depencies
  "std_msgs"
  "geometry_msgs")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  ${msg_file_depencies}
)

ament_export_dependencies(rosidl_default_runtime)


rosidl_target_interfaces (mocapInterface ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_generators)

target_link_libraries(
  mocapInterface ${rclcpp_LIBRARIES} ${OpenCV_LIBS}
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
