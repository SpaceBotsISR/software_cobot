cmake_minimum_required(VERSION 2.8.3)
project(SCobot_MotionSub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  scobot_msgs
)

catkin_package(
        CATKIN_DEPENDS
        #LIBRARIES SCobot_MotionSub # lastly added - not sure
        roscpp
        scobot_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/motion.cpp src/SCobotPipeCommunication.cpp )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


