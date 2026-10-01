#ifndef MY_ROBOT_SORTING__SORTING_TYPES_HPP_
#define MY_ROBOT_SORTING__SORTING_TYPES_HPP_

#include <string>

#include <my_robot_interfaces/msg/detected_object.hpp>

namespace my_robot_sorting
{

struct TaskPlan
{
  my_robot_interfaces::msg::DetectedObject grasp;
  my_robot_interfaces::msg::DetectedObject target;
};

struct Step
{
  enum class Type
  {
    kPose,
    kGripper
  };

  Type type{Type::kPose};

  double x{0.0};
  double y{0.0};
  double z{0.0};

  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};

  bool cartesian_path{false};

  bool open_gripper{false};

  double wait_seconds{0.0};

  std::string description;
};

}  // namespace my_robot_sorting

#endif  // MY_ROBOT_SORTING__SORTING_TYPES_HPP_
