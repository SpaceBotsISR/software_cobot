cmake_minimum_required(VERSION 3.8)
project(delay_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)


include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}  
)

add_executable(delay_generator
  src/delay_generator.cpp
)

ament_target_dependencies(delay_generator 
  std_msgs
  rclcpp 
)
target_link_libraries(delay_generator nlohmann_json::nlohmann_json) # The library or executable that require yaml-cpp library

target_include_directories(delay_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(delay_generator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++1
  ament_target_dependencies(
  delay_generator
  "std_msgs"
)

install(TARGETS delay_generator
  DESTINATION lib/${PROJECT_NAME})

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
