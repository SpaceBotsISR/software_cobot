cmake_minimum_required(VERSION 3.8)
project(mav_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(trajectory_msgs REQUIRED) 
find_package(nav_msgs REQUIRED)
find_package(Eigen3 QUIET)
find_package(ament_cmake REQUIRED)
 find_package(rosidl_default_generators REQUIRED)

include_directories(include)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

if(NOT EIGEN3_FOUND)
  # ROS == Indigo.
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  # ROS > Indigo.
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set (msg_files 
  "msg/Actuators.msg"
  "msg/AttitudeThrust.msg"
  "msg/RateThrust.msg"
  "msg/RollPitchYawrateThrust.msg"
  "msg/TorqueThrust.msg"
  "msg/Status.msg"
  "msg/FilteredSensorData.msg"
  "msg/GpsWaypoint.msg"
)

set (msg_file_depencies
  std_msgs
  geometry_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  ${msg_file_depencies}
)

ament_export_dependencies(rosidl_default_generators)

install(
  DIRECTORY include/
  DESTINATION include
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