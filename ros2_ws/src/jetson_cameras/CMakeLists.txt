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
<<<<<<< HEAD
=======
find_package(cv_bridge REQUIRED)
>>>>>>> camera ros2
find_package(OpenCV REQUIRED)
=======
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)
>>>>>>> camera_publisher now compiles

include_directories(
  include
)

add_executable(camera_publisher src/camera_publisher.cpp)
<<<<<<< HEAD
<<<<<<< HEAD
ament_target_dependencies(camera_publisher rclcpp sensor_msgs)
target_link_libraries(camera_publisher ${OpenCV_LIBS}) # Change this line
=======
ament_target_dependencies(camera_publisher rclcpp sensor_msgs cv_bridge OpenCV)
>>>>>>> camera ros2
=======
ament_target_dependencies(camera_publisher rclcpp sensor_msgs)
>>>>>>> camera_publisher now compiles

install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
