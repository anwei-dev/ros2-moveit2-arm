#ifndef MY_ROBOT_SORTING__COLOR_SORTING_NODE_HPP_
#define MY_ROBOT_SORTING__COLOR_SORTING_NODE_HPP_

#include <memory>
#include <vector>

#include <example_interfaces/msg/bool.hpp>
#include <my_robot_interfaces/action/move_to_pose.hpp>
#include <my_robot_interfaces/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "my_robot_sorting/sorting_types.hpp"
#include "my_robot_sorting/task_planner.hpp"

class ColorSortingNode : public rclcpp::Node
{
public:
  ColorSortingNode();

private:
  using MoveToPose = my_robot_interfaces::action::MoveToPose;

  using GoalHandleMoveToPose =
    rclcpp_action::ClientGoalHandle<MoveToPose>;

  using Trigger = std_srvs::srv::Trigger;

  void objectsCallback(
    const my_robot_interfaces::msg::DetectedObjectArray::SharedPtr msg);

  void tryStartSequence(
    const my_robot_interfaces::msg::DetectedObjectArray &msg);

  void runNextStep();

  void sendPoseGoal(
    const my_robot_sorting::Step &step,
    bool use_cartesian);

  void goalResponseCallback(
    const GoalHandleMoveToPose::SharedPtr &goal_handle);

  void feedbackCallback(
    GoalHandleMoveToPose::SharedPtr goal_handle,
    const std::shared_ptr<const MoveToPose::Feedback> feedback);

  void resultCallback(
    const GoalHandleMoveToPose::WrappedResult &result);

  void updatePlanningScene();

private:
  rclcpp_action::Client<MoveToPose>::SharedPtr
    move_to_pose_client_;

  rclcpp::Client<Trigger>::SharedPtr
    update_planning_scene_client_;

  rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr
    gripper_pub_;

  rclcpp::Subscription<
    my_robot_interfaces::msg::DetectedObjectArray>::SharedPtr
    object_sub_;

  rclcpp::TimerBase::SharedPtr
    step_timer_;

  std::shared_ptr<TaskPlanner>
    task_planner_;

  std::vector<my_robot_sorting::Step>
    steps_;

  bool sequence_started_{false};
  bool step_in_progress_{false};

  std::size_t current_step_index_{0};

  rclcpp::Time next_step_ready_time_{
    0, 0, RCL_ROS_TIME};

  bool wait_for_stable_detections_{true};
  int stable_detection_count_{3};

  // 当前步骤是否已经进行过
  // “笛卡尔路径失败 -> 普通规划” 的 fallback
  bool cartesian_fallback_attempted_{false};
};

#endif  // MY_ROBOT_SORTING__COLOR_SORTING_NODE_HPP_