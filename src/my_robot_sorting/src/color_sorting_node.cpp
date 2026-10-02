#include "my_robot_sorting/color_sorting_node.hpp"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

ColorSortingNode::ColorSortingNode()
: Node("color_sorting_node")
{
  move_to_pose_client_ =
    rclcpp_action::create_client<MoveToPose>(
      this,
      "move_to_pose");

  gripper_pub_ =
    create_publisher<example_interfaces::msg::Bool>(
      "open_gripper",
      10);

  update_planning_scene_client_ =
    create_client<Trigger>(
      "/update_planning_scene");

  object_sub_ =
    create_subscription<
      my_robot_interfaces::msg::DetectedObjectArray>(
      "/detected_objects",
      10,
      std::bind(
        &ColorSortingNode::objectsCallback,
        this,
        std::placeholders::_1));

  const double pregrasp_height =
    declare_parameter("pregrasp_height", 0.20);

  const double grasp_surface_offset =
    declare_parameter("grasp_surface_offset", 0.015);

  const double lift_height =
    declare_parameter("lift_height", 0.20);

  const double preplace_clearance =
    declare_parameter("preplace_clearance", 0.23);

  const double place_surface_offset =
    declare_parameter("place_surface_offset", 0.13);

  const double retreat_height =
    declare_parameter("retreat_height", 0.35);

  const double end_x =
    declare_parameter("end_x", 0.0);

  const double end_y =
    declare_parameter("end_y", 0.8);

  const double roll =
    declare_parameter("roll", 3.14);

  const double pitch =
    declare_parameter("pitch", 0.0);

  const double yaw =
    declare_parameter("yaw", 0.0);

  const double pose_wait_sec =
    declare_parameter("pose_wait_sec", 5.0);

  const double cartesian_wait_sec =
    declare_parameter("cartesian_wait_sec", 3.0);

  const double gripper_wait_sec =
    declare_parameter("gripper_wait_sec", 0.7);

  wait_for_stable_detections_ =
    declare_parameter(
      "wait_for_stable_detections",
      true);

  stable_detection_count_ =
    declare_parameter(
      "stable_detection_count",
      3);

  task_planner_ =
    std::make_shared<TaskPlanner>(
      pregrasp_height,
      grasp_surface_offset,
      lift_height,
      preplace_clearance,
      place_surface_offset,
      retreat_height,
      end_x,
      end_y,
      roll,
      pitch,
      yaw,
      pose_wait_sec,
      cartesian_wait_sec,
      gripper_wait_sec);

  step_timer_ =
    create_wall_timer(
      10ms,
      std::bind(
        &ColorSortingNode::runNextStep,
        this));

  step_timer_->cancel();

  RCLCPP_INFO(
    get_logger(),
    "Waiting for detected objects to generate a sorting sequence");
}

void ColorSortingNode::objectsCallback(
  const my_robot_interfaces::msg::DetectedObjectArray::SharedPtr msg)
{
  if (sequence_started_)
  {
    return;
  }

  if (!wait_for_stable_detections_)
  {
    tryStartSequence(*msg);
    return;
  }

  task_planner_->updateDetectionState(*msg);

  if (task_planner_->hasStableDetection(
        stable_detection_count_))
  {
    tryStartSequence(*msg);
  }
}

void ColorSortingNode::tryStartSequence(
  const my_robot_interfaces::msg::DetectedObjectArray &msg)
{
  const auto task_plan =
    task_planner_->selectTask(msg);

  if (!task_plan.has_value())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "No valid grasp/target pair found in /detected_objects");

    return;
  }

  steps_ =
    task_planner_->buildSequence(*task_plan);

  if (steps_.empty())
  {
    RCLCPP_WARN(
      get_logger(),
      "Generated empty sorting sequence");

    return;
  }

  sequence_started_ = true;
  current_step_index_ = 0;

  RCLCPP_INFO(
      get_logger(),
      "Starting sorting sequence, "
      "requesting Planning Scene update");

  updatePlanningScene();

  next_step_ready_time_ = now();

  step_timer_->reset();

  RCLCPP_INFO(
    get_logger(),
    "Starting sorting sequence: "
    "grasp=%s color=%s target=%s color=%s",
    task_plan->grasp.id.c_str(),
    task_plan->grasp.color.c_str(),
    task_plan->target.id.c_str(),
    task_plan->target.color.c_str());
}

void ColorSortingNode::sendPoseGoal(
  const my_robot_sorting::Step &step)
{
  if (!move_to_pose_client_->wait_for_action_server(1s))
  {
    RCLCPP_ERROR(
      get_logger(),
      "MoveToPose action server is not available");

    sequence_started_ = false;
    step_timer_->cancel();

    return;
  }

  MoveToPose::Goal goal;

  goal.x = step.x;
  goal.y = step.y;
  goal.z = step.z;

  goal.roll = step.roll;
  goal.pitch = step.pitch;
  goal.yaw = step.yaw;

  goal.cartesian_path = step.cartesian_path;

  RCLCPP_INFO(
    get_logger(),
    "Step %zu/%zu: %s -> send MoveToPose goal "
    "pose(%.3f, %.3f, %.3f) cartesian=%s",
    current_step_index_ + 1,
    steps_.size(),
    step.description.c_str(),
    step.x,
    step.y,
    step.z,
    step.cartesian_path ? "true" : "false");

  step_in_progress_ = true;

  rclcpp_action::Client<MoveToPose>::SendGoalOptions options;

  options.goal_response_callback =
    std::bind(
      &ColorSortingNode::goalResponseCallback,
      this,
      std::placeholders::_1);

  options.feedback_callback =
    std::bind(
      &ColorSortingNode::feedbackCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2);

  options.result_callback =
    std::bind(
      &ColorSortingNode::resultCallback,
      this,
      std::placeholders::_1);

  move_to_pose_client_->async_send_goal(
    goal,
    options);
}

void ColorSortingNode::goalResponseCallback(
  const GoalHandleMoveToPose::SharedPtr &goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(
      get_logger(),
      "MoveToPose goal was rejected by the action server");

    step_in_progress_ = false;
    sequence_started_ = false;
    step_timer_->cancel();

    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "MoveToPose goal accepted by the action server");
}

void ColorSortingNode::feedbackCallback(
  GoalHandleMoveToPose::SharedPtr,
  const std::shared_ptr<const MoveToPose::Feedback> feedback)
{
  RCLCPP_INFO(
    get_logger(),
    "MoveToPose feedback: state=%s, progress=%.2f",
    feedback->state.c_str(),
    feedback->progress);
}

void ColorSortingNode::resultCallback(
  const GoalHandleMoveToPose::WrappedResult &result)
{
  step_in_progress_ = false;

  if (result.code ==
      rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->success)
  {
    RCLCPP_INFO(
      get_logger(),
      "MoveToPose succeeded: error_code=%d",
      result.result->error_code);

    ++current_step_index_;

    next_step_ready_time_ =
      now() + rclcpp::Duration::from_seconds(0.0);

    return;
  }

  RCLCPP_ERROR(
    get_logger(),
    "MoveToPose failed: result_code=%d, "
    "success=%s, error_code=%d",
    static_cast<int>(result.code),
    result.result->success ? "true" : "false",
    result.result->error_code);

  sequence_started_ = false;
  step_timer_->cancel();
}

void ColorSortingNode::updatePlanningScene()
{
  if (!update_planning_scene_client_->service_is_ready())
  {
    RCLCPP_WARN(
      get_logger(),
      "Planning Scene update service "
      "'/update_planning_scene' is not available");

    return;
  }

  auto request =
    std::make_shared<Trigger::Request>();

  update_planning_scene_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<Trigger>::SharedFuture future)
    {
      try
      {
        const auto response = future.get();

        if (response->success)
        {
          RCLCPP_INFO(
            get_logger(),
            "Planning Scene update succeeded: %s",
            response->message.c_str());
        }
        else
        {
          RCLCPP_WARN(
            get_logger(),
            "Planning Scene update failed: %s",
            response->message.c_str());
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Exception while updating Planning Scene: %s",
          e.what());
      }
    });
}

void ColorSortingNode::runNextStep()
{
  if (!sequence_started_)
  {
    return;
  }

  if (step_in_progress_)
  {
    return;
  }

  if (now() < next_step_ready_time_)
  {
    return;
  }

  if (current_step_index_ >= steps_.size())
  {
    RCLCPP_INFO(
        get_logger(),
        "Sorting sequence completed");

    sequence_started_ = false;
    step_timer_->cancel();

    return;
  }

  const auto &step =
    steps_[current_step_index_];

  if (step.type ==
      my_robot_sorting::Step::Type::kPose)
  {
    sendPoseGoal(step);
    return;
  }

  example_interfaces::msg::Bool command;

  command.data = step.open_gripper;

  gripper_pub_->publish(command);

  RCLCPP_INFO(
    get_logger(),
    "Step %zu/%zu: %s -> open_gripper=%s",
    current_step_index_ + 1,
    steps_.size(),
    step.description.c_str(),
    step.open_gripper ? "true" : "false");

  ++current_step_index_;

  next_step_ready_time_ =
    now() +
    rclcpp::Duration::from_seconds(
      step.wait_seconds);
}
