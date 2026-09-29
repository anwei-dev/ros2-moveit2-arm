cmake_minimum_required(VERSION 3.8)
project(my_robot_vision_pcl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(point_cloud_processor_node
  src/point_cloud_processor_node.cpp
  src/point_cloud_processor.cpp
)

ament_target_dependencies(point_cloud_processor_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(point_cloud_processor_node
  ${PCL_LIBRARIES}
)

install(
  TARGETS point_cloud_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()