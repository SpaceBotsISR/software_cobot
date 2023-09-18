cmake_minimum_required(VERSION 2.8.3)
project(SCobot_AttitudePub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ROS_Publisher
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/attitudePublisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})