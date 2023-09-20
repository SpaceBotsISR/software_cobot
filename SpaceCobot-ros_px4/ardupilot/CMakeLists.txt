cmake_minimum_required(VERSION 3.7)
project(ardupilot)
# we just grab all the cpp and h files. not ideal, but works well enough
# means a "reload CMake project" when you add new files
#file(GLOB_RECURSE SOURCE_FILES APMrover2/*.cpp APMrover2/*.h)
#file(GLOB_RECURSE SOURCE_FILES ArduCopter/*.cpp ArduCopter/*.h)
#file(GLOB_RECURSE SOURCE_FILES ArduPlane/*.cpp ArduPlane/*.h)
#file(GLOB_RECURSE SOURCE_FILES ArduSpaceCRAP/*.cpp ArduSpaceCRAP/*.h)
file(GLOB_RECURSE SOURCE_FILES libraries/*.cpp libraries/*.h)
file(GLOB_RECURSE SOURCE_FILES modules/PX4Firmware/mavlink/include/mavlink/*.h modules/PX4Firmware/mavlink/include/mavlink/*.cpp)
#include_directories(APMrover2)
#include_directories(ArduCopter)
#include_directories(ArduSpaceCRAP)
include_directories(SpaceCobot)
include_directories(libraries)
include_directories(modules/PX4Firmware/mavlink/include/mavlink)
# To work with SITL
add_definitions(-DSKETCHBOOK="./")
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_SITL)
#add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_PX4)
add_executable(fake-target-for-clion ${SOURCE_FILES} SpaceCobot/Copter.cpp SpaceCobot/Copter.h SpaceCobot/config.h SpaceCobot/defines.h SpaceCobot/SCobot_Config.h SpaceCobot/SpaceCobot.cpp SpaceCobot/system.cpp SpaceCobot/PC_PositionControl_SCobot.cpp SpaceCobot/PC_PositionControl_SCobot.h SpaceCobot/SCobot_PipeMsgs.h SpaceCobot/SCobot_CSVDebug.cpp)

#Could not find a way to include mavlink message defenitions on modules/mavlink/message_definitions/v1.0/common.xml
