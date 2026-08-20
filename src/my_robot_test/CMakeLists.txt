cmake_minimum_required(VERSION 3.8)
project(my_robot_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(point_cloud_box_center
  src/point_cloud_box_center.cpp
)

ament_target_dependencies(point_cloud_box_center
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  point_cloud_box_center
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
