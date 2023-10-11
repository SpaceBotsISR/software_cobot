cmake_minimum_required(VERSION 3.8)
project(jetson_cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
<<<<<<< HEAD
=======
find_package(cv_bridge REQUIRED)
>>>>>>> camera ros2
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_publisher src/camera_publisher.cpp)
<<<<<<< HEAD
ament_target_dependencies(camera_publisher rclcpp sensor_msgs)
target_link_libraries(camera_publisher ${OpenCV_LIBS}) # Change this line
=======
ament_target_dependencies(camera_publisher rclcpp sensor_msgs cv_bridge OpenCV)
>>>>>>> camera ros2

install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
