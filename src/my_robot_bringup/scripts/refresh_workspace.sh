#!/usr/bin/env bash
set -euo pipefail

WS="/home/anl/git/bot/ros2_ws"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# Stop common ROS/Gazebo processes to avoid stale state.
pkill -f gazebo || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f rviz2 || true
pkill -f ros2_control_node || true
pkill -f spawner || true
pkill -f my_robot_commander_cpp || true

cd "$WS"

# Clean only related package artifacts + logs.
rm -rf \
  build/my_robot_description \
  build/my_robot_bringup \
  build/my_robot_moveit_config \
  build/my_robot_commander_cpp \
  install/my_robot_description \
  install/my_robot_bringup \
  install/my_robot_moveit_config \
  install/my_robot_commander_cpp \
  log

# Avoid nounset errors from ROS setup scripts under `set -u`.
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${COLCON_TRACE:=}"
# ROS setup scripts may reference unset variables; source under relaxed nounset.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

colcon build \
  --symlink-install \
  --packages-select \
  my_robot_description \
  my_robot_moveit_config \
  my_robot_commander_cpp \
  my_robot_bringup

set +u
source "$WS/install/setup.bash"
set -u

echo "Workspace refreshed."
echo "Launch manually when needed:"
echo "  ros2 launch my_robot_bringup robot.launch.py"
