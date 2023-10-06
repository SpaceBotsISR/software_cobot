cmake_minimum_required(VERSION 3.8)

project(space_cobot_interface)

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
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

set(msg_files
  "msg/PwmValues.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include /opt/ros/humble/include/std_msgs)
include_directories(include /opt/ros/humble/include/mavros_msgs)
include_directories(include /opt/ros/humble/include/builtin_interfaces)
include_directories(include /usr/include/opencv4)

add_executable(spaceCobotInterface
  src/space_cobot_interface.cpp
  src/OpenLoop.cpp
  src/AMatrix.cpp
  src/Actuation.cpp
)

target_include_directories(spaceCobotInterface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  spaceCobotInterface
  rclcpp
  geometry_msgs
  sensor_msgs
  mavros_msgs
  std_msgs
  tf2
  tf2_ros
)

rosidl_target_interfaces (spaceCobotInterface ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(
  spaceCobotInterface ${colcon_LIBRARIES} ${OpenCV_LIBS}
)
  
install(TARGETS spaceCobotInterface
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