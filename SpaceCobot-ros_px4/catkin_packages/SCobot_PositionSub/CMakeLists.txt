cmake_minimum_required(VERSION 2.8.3)
project(SCobot_PositionSub)

find_package(catkin REQUIRED
        roscpp
        std_msgs
        scobot_msgs
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        scobot_msgs
)
## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/positionSubcriber.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

