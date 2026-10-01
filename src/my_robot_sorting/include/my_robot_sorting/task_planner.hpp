#ifndef MY_ROBOT_SORTING__TASK_PLANNER_HPP_
#define MY_ROBOT_SORTING__TASK_PLANNER_HPP_

#include <optional>
#include <string>
#include <vector>

#include <my_robot_interfaces/msg/detected_object_array.hpp>

#include "my_robot_sorting/sorting_types.hpp"

class TaskPlanner
{
public:
  TaskPlanner(
    double pregrasp_height,
    double grasp_surface_offset,
    double lift_height,
    double preplace_clearance,
    double place_surface_offset,
    double retreat_height,
    double end_x,
    double end_y,
    double roll,
    double pitch,
    double yaw,
    double pose_wait_sec,
    double cartesian_wait_sec,
    double gripper_wait_sec);

  bool isDetectionStable(
    const my_robot_interfaces::msg::DetectedObjectArray &msg) const;

  void updateDetectionState(
    const my_robot_interfaces::msg::DetectedObjectArray &msg);

  bool hasStableDetection(int required_count) const;

  std::optional<my_robot_sorting::TaskPlan> selectTask(
    const my_robot_interfaces::msg::DetectedObjectArray &msg) const;

  std::vector<my_robot_sorting::Step> buildSequence(
    const my_robot_sorting::TaskPlan &task_plan) const;

private:
  std::string buildSignature(
    const my_robot_interfaces::msg::DetectedObjectArray &msg) const;

  my_robot_sorting::Step makePoseStep(
    double x,
    double y,
    double z,
    bool cartesian_path,
    double wait_seconds,
    const std::string &description) const;

  my_robot_sorting::Step makeGripperStep(
    bool open_gripper,
    double wait_seconds,
    const std::string &description) const;

private:
  double pregrasp_height_;
  double grasp_surface_offset_;
  double lift_height_;
  double preplace_clearance_;
  double place_surface_offset_;
  double retreat_height_;

  double end_x_;
  double end_y_;

  double roll_;
  double pitch_;
  double yaw_;

  double pose_wait_sec_;
  double cartesian_wait_sec_;
  double gripper_wait_sec_;

  std::string stable_signature_;
  int stable_hits_{0};
};

#endif  // MY_ROBOT_SORTING__TASK_PLANNER_HPP_
