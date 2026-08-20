cmake_minimum_required(VERSION 3.8)
project(attach)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_model_attachment_plugin_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(auto_attach_node
  src/auto_attach_node.cpp
)

ament_target_dependencies(auto_attach_node
  example_interfaces
  gazebo_msgs
  gazebo_model_attachment_plugin_msgs
  rclcpp
)

install(TARGETS
  auto_attach_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
