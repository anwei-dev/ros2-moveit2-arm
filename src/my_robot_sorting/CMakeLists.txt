cmake_minimum_required(VERSION 3.8)
project(my_robot_sorting)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(color_sorting_node
  src/color_sorting_node.cpp
  src/main.cpp
  src/task_planner.cpp
)

ament_target_dependencies(color_sorting_node
  example_interfaces
  my_robot_interfaces
  rclcpp_action
  rclcpp
  std_srvs
)

target_include_directories(color_sorting_node PRIVATE
  include
)

install(TARGETS
  color_sorting_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
