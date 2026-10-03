#include "my_robot_sorting/task_planner.hpp"

#include <cmath>
#include <utility>

TaskPlanner::TaskPlanner(
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
  double gripper_wait_sec)
: pregrasp_height_(pregrasp_height),
  grasp_surface_offset_(grasp_surface_offset),
  lift_height_(lift_height),
  preplace_clearance_(preplace_clearance),
  place_surface_offset_(place_surface_offset),
  retreat_height_(retreat_height),
  end_x_(end_x),
  end_y_(end_y),
  roll_(roll),
  pitch_(pitch),
  yaw_(yaw),
  pose_wait_sec_(pose_wait_sec),
  cartesian_wait_sec_(cartesian_wait_sec),
  gripper_wait_sec_(gripper_wait_sec)
{
}

bool TaskPlanner::isDetectionStable(
  const my_robot_interfaces::msg::DetectedObjectArray &msg) const
{
  return buildSignature(msg) == stable_signature_;
}

void TaskPlanner::updateDetectionState(
  const my_robot_interfaces::msg::DetectedObjectArray &msg)
{
  if (isDetectionStable(msg))
  {
    ++stable_hits_;
  }
  else
  {
    stable_hits_ = 1;
    stable_signature_ = buildSignature(msg);
  }
}

bool TaskPlanner::hasStableDetection(int required_count) const
{
  return stable_hits_ >= required_count;
}

std::string TaskPlanner::buildSignature(
  const my_robot_interfaces::msg::DetectedObjectArray &msg) const
{
  std::string signature;

  for (const auto &object : msg.objects)
  {
    signature += object.color + "|" + object.shape + "|";

    signature +=
      std::to_string(
      std::lround(object.top_center.x * 100.0));

    signature += ",";

    signature +=
      std::to_string(
      std::lround(object.top_center.y * 100.0));

    signature += ";";
  }

  return signature;
}

std::optional<my_robot_sorting::TaskPlan> TaskPlanner::selectTask(
  const my_robot_interfaces::msg::DetectedObjectArray &msg) const
{
  for (const auto &grasp_candidate : msg.objects)
  {
    if (grasp_candidate.shape != "grasp")
    {
      continue;
    }

    for (const auto &target_candidate : msg.objects)
    {
      if (target_candidate.shape != "target")
      {
        continue;
      }

      if (target_candidate.color != grasp_candidate.color)
      {
        continue;
      }

      return my_robot_sorting::TaskPlan{
        grasp_candidate,
        target_candidate
      };
    }
  }

  return std::nullopt;
}

std::vector<my_robot_sorting::Step> TaskPlanner::buildSequence(
  const my_robot_sorting::TaskPlan &task_plan) const
{
  std::vector<my_robot_sorting::Step> steps;

  const auto &grasp = task_plan.grasp;
  const auto &target = task_plan.target;

  const double grasp_z =
    grasp.height + grasp_surface_offset_;

  const double pregrasp_z =
    grasp.height + pregrasp_height_;

  const double lifted_z =
    grasp.height + lift_height_;

  const double preplace_z =
    target.height + preplace_clearance_;

  const double place_z =
    target.height + place_surface_offset_;

  steps.push_back(
    makeGripperStep(
      true,
      gripper_wait_sec_,
      "open gripper"));

  steps.push_back(
    makePoseStep(
      grasp.top_center.x,
      grasp.top_center.y,
      pregrasp_z,
      false,
      pose_wait_sec_,
      "move to pregrasp"));

  steps.push_back(
    makePoseStep(
      grasp.top_center.x,
      grasp.top_center.y,
      grasp_z,
      true,
      cartesian_wait_sec_,
      "descend to grasp"));

  steps.push_back(
    makeGripperStep(
      false,
      gripper_wait_sec_,
      "close gripper"));

  steps.push_back(
    makePoseStep(
      grasp.top_center.x,
      grasp.top_center.y,
      lifted_z,
      true,
      cartesian_wait_sec_,
      "lift object"));

  steps.push_back(
    makePoseStep(
      target.top_center.x,
      target.top_center.y,
      preplace_z,
      false,
      pose_wait_sec_,
      "move to preplace"));

  steps.push_back(
    makePoseStep(
      target.top_center.x,
      target.top_center.y,
      place_z,
      true,
      cartesian_wait_sec_,
      "descend to place"));

  steps.push_back(
    makeGripperStep(
      true,
      gripper_wait_sec_,
      "open gripper to release"));

  steps.push_back(
    makePoseStep(
      target.top_center.x,
      target.top_center.y,
      preplace_z,
      true,
      cartesian_wait_sec_,
      "retreat from target"));

  steps.push_back(
    makePoseStep(
      end_x_,
      end_y_,
      retreat_height_ + 0.3,
      false,
      pose_wait_sec_,
      "move to end pose"));

  return steps;
}

my_robot_sorting::Step TaskPlanner::makePoseStep(
  double x,
  double y,
  double z,
  bool cartesian_path,
  double wait_seconds,
  const std::string &description) const
{
  my_robot_sorting::Step step;

  step.type = my_robot_sorting::Step::Type::kPose;

  step.x = x;
  step.y = y;
  step.z = z;

  step.roll = roll_;
  step.pitch = pitch_;
  step.yaw = yaw_;

  step.cartesian_path = cartesian_path;

  step.wait_seconds = wait_seconds;
  step.description = description;

  return step;
}

my_robot_sorting::Step TaskPlanner::makeGripperStep(
  bool open_gripper,
  double wait_seconds,
  const std::string &description) const
{
  my_robot_sorting::Step step;

  step.type = my_robot_sorting::Step::Type::kGripper;

  step.open_gripper = open_gripper;
  step.wait_seconds = wait_seconds;
  step.description = description;

  return step;
}
