cmake_minimum_required(VERSION 2.8.3)
project(mocap_interface)

add_definitions(-std=c++11)

find_package(Mavlink QUIET)

# We still have to call catkin package for this to be a valid package,
# even if empty.
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  mav_msgs
  roscpp
  sensor_msgs
  message_generation
        message_runtime
  #rotors_comm
  tf
)

find_package(Eigen3 REQUIRED)

add_message_files(
        FILES
        mocap_msg.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)


catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES 
  CATKIN_DEPENDS geometry_msgs mav_msgs mavros mavros_msgs roscpp sensor_msgs message_generation message_runtime
  DEPENDS
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
)

# MOTION CAPTURE SYSTEM INTERFACE ----------------------------------

add_executable(mocap_interface src/nodes/mocap_interface.cpp)
add_dependencies(mocap_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(mocap_interface mocap_interface_generate_messages_cpp)
target_link_libraries(mocap_interface ${catkin_LIBRARIES})

install(TARGETS
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS mocap_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

