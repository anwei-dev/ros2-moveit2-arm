cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(commander
  src/commander_template.cpp
)

add_executable(planning_scene_manager
  src/planning_scene_manager.cpp
)

target_include_directories(commander PRIVATE
  include
)

target_include_directories(planning_scene_manager PRIVATE
  include
)

ament_target_dependencies(planning_scene_manager
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
  my_robot_interfaces
)

ament_target_dependencies(commander
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  example_interfaces
  my_robot_interfaces
  sensor_msgs
  std_msgs
)

install(TARGETS
  commander
  planning_scene_manager
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
