#ifndef COLOR_POINT_CLOUD_DETECTOR__DETECTION_STABILIZER_HPP_
#define COLOR_POINT_CLOUD_DETECTOR__DETECTION_STABILIZER_HPP_

#include <my_robot_interfaces/msg/detected_object.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

class DetectionStabilizer
{
public:
  explicit DetectionStabilizer(
    double required_stable_time = 5.0);

  bool update(
    const std::vector<
      my_robot_interfaces::msg::DetectedObject> &objects,
    const rclcpp::Time &now);

  bool isStable() const;

  const std::vector<
    my_robot_interfaces::msg::DetectedObject> &
  getStableObjects() const;

private:
  bool objectsStable(
    const std::vector<
      my_robot_interfaces::msg::DetectedObject> &a,
    const std::vector<
      my_robot_interfaces::msg::DetectedObject> &b) const;

  double required_stable_time_;

  static constexpr double kPositionThreshold = 0.005;
  static constexpr double kSizeThreshold = 0.005;

  rclcpp::Time stable_start_time_;

  std::vector<
    my_robot_interfaces::msg::DetectedObject>
    stable_candidate_objects_;

  bool has_stable_candidate_ = false;
  bool is_detection_stable_ = false;
};

#endif