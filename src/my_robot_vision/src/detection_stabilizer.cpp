#include "detection_stabilizer.hpp"

#include <cmath>

DetectionStabilizer::DetectionStabilizer(
  double required_stable_time)
: required_stable_time_(required_stable_time)
{
}

bool DetectionStabilizer::update(
  const std::vector<
    my_robot_interfaces::msg::DetectedObject> &objects,
  const rclcpp::Time &now)
{
  if (!has_stable_candidate_)
  {
    stable_candidate_objects_ = objects;
    stable_start_time_ = now;
    has_stable_candidate_ = true;
    is_detection_stable_ = false;

    return false;
  }

  if (!objectsStable(
      stable_candidate_objects_,
      objects))
  {
    stable_candidate_objects_ = objects;
    stable_start_time_ = now;
    is_detection_stable_ = false;

    return false;
  }

  if (!is_detection_stable_)
  {
    const double stable_duration =
      (now - stable_start_time_).seconds();

    if (stable_duration < required_stable_time_)
    {
      return false;
    }

    is_detection_stable_ = true;
  }

  return true;
}

bool DetectionStabilizer::isStable() const
{
  return is_detection_stable_;
}

const std::vector<
  my_robot_interfaces::msg::DetectedObject> &
DetectionStabilizer::getStableObjects() const
{
  return stable_candidate_objects_;
}

bool DetectionStabilizer::objectsStable(
  const std::vector<
    my_robot_interfaces::msg::DetectedObject> &a,
  const std::vector<
    my_robot_interfaces::msg::DetectedObject> &b) const
{
  if (a.size() != b.size())
  {
    return false;
  }

  for (std::size_t i = 0; i < a.size(); ++i)
  {
    const auto &obj_a = a[i];
    const auto &obj_b = b[i];

    if (obj_a.color != obj_b.color)
    {
      return false;
    }

    if (obj_a.shape != obj_b.shape)
    {
      return false;
    }

    if (std::abs(
        obj_a.top_center.x -
        obj_b.top_center.x) >
        kPositionThreshold)
    {
      return false;
    }

    if (std::abs(
        obj_a.top_center.y -
        obj_b.top_center.y) >
        kPositionThreshold)
    {
      return false;
    }

    if (std::abs(
        obj_a.top_center.z -
        obj_b.top_center.z) >
        kPositionThreshold)
    {
      return false;
    }

    if (std::abs(
        obj_a.height -
        obj_b.height) >
        kSizeThreshold)
    {
      return false;
    }

    if (std::abs(
        obj_a.diameter_x -
        obj_b.diameter_x) >
        kSizeThreshold)
    {
      return false;
    }
  }

  return true;
}