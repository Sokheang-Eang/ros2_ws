cmake_minimum_required(VERSION 3.8)
project(cpp_script)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find required library
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# add executable for script
add_executable(twist_subscriber_node src/read_data_twist.cpp)
# build
ament_target_dependencies(twist_subscriber_node rclcpp geometry_msgs)
# call package
install(TARGETS
  twist_subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

