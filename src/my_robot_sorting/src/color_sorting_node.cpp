#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <example_interfaces/msg/bool.hpp>
#include <my_robot_interfaces/msg/detected_object.hpp>
#include <my_robot_interfaces/msg/detected_object_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class ColorSortingNode : public rclcpp::Node
{
public:
  ColorSortingNode()
  : Node("color_sorting_node")
  {
    pose_pub_ = create_publisher<my_robot_interfaces::msg::PoseCommand>("pose_command", 10);
    gripper_pub_ = create_publisher<example_interfaces::msg::Bool>("open_gripper", 10);

    object_sub_ = create_subscription<my_robot_interfaces::msg::DetectedObjectArray>(
      "/detected_objects", 10,
      std::bind(&ColorSortingNode::objectsCallback, this, std::placeholders::_1));

    pregrasp_height_ = declare_parameter("pregrasp_height", 0.20);
    grasp_surface_offset_ = declare_parameter("grasp_surface_offset", 0.02);
    lift_height_ = declare_parameter("lift_height", 0.20);
    preplace_clearance_ = declare_parameter("preplace_clearance", 0.23);
    place_surface_offset_ = declare_parameter("place_surface_offset", 0.13);
    retreat_height_ = declare_parameter("retreat_height", 0.35);
    end_x_ = declare_parameter("end_x", 0.0);
    end_y_ = declare_parameter("end_y", 0.8);
    roll_ = declare_parameter("roll", 3.14);
    pitch_ = declare_parameter("pitch", 0.0);
    yaw_ = declare_parameter("yaw", 0.0);
    pose_wait_sec_ = declare_parameter("pose_wait_sec", 5.0);
    cartesian_wait_sec_ = declare_parameter("cartesian_wait_sec", 3.0);
    gripper_wait_sec_ = declare_parameter("gripper_wait_sec", 2.0);
    wait_for_stable_detections_ = declare_parameter("wait_for_stable_detections", true);
    stable_detection_count_ = declare_parameter("stable_detection_count", 3);

    step_timer_ = create_wall_timer(100ms, std::bind(&ColorSortingNode::runNextStep, this));
    step_timer_->cancel();

    RCLCPP_INFO(get_logger(), "Waiting for detected objects to generate a sorting sequence");
  }

private:
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

    Type type;
    my_robot_interfaces::msg::PoseCommand pose;
    bool open_gripper{false};
    double wait_seconds{0.0};
    std::string description;
  };

  void objectsCallback(const my_robot_interfaces::msg::DetectedObjectArray::SharedPtr msg)
  {
    latest_objects_ = msg;
    if (sequence_started_) {
      return;
    }

    if (!wait_for_stable_detections_) {
      tryStartSequence(*msg);
      return;
    }

    if (isDetectionStable(*msg)) {
      ++stable_hits_;
    } else {
      stable_hits_ = 1;
      stable_signature_ = buildSignature(*msg);
    }

    if (stable_hits_ >= stable_detection_count_) {
      tryStartSequence(*msg);
    }
  }

  bool isDetectionStable(const my_robot_interfaces::msg::DetectedObjectArray & msg) const
  {
    return buildSignature(msg) == stable_signature_;
  }

  std::string buildSignature(const my_robot_interfaces::msg::DetectedObjectArray & msg) const
  {
    std::string signature;
    for (const auto & object : msg.objects) {
      signature += object.color + "|" + object.shape + "|";
      signature += std::to_string(std::lround(object.top_center.x * 100.0)) + ",";
      signature += std::to_string(std::lround(object.top_center.y * 100.0)) + ";";
    }
    return signature;
  }

  void tryStartSequence(const my_robot_interfaces::msg::DetectedObjectArray & msg)
  {
    const auto task_plan = selectTask(msg);
    if (!task_plan.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No valid grasp/target pair found in /detected_objects");
      return;
    }

    buildSequence(*task_plan);
    if (steps_.empty()) {
      RCLCPP_WARN(get_logger(), "Generated empty sorting sequence");
      return;
    }

    sequence_started_ = true;
    current_step_index_ = 0;
    next_step_ready_time_ = now();
    step_timer_->reset();

    RCLCPP_INFO(
      get_logger(),
      "Starting sorting sequence: grasp=%s color=%s target=%s color=%s",
      task_plan->grasp.id.c_str(), task_plan->grasp.color.c_str(),
      task_plan->target.id.c_str(), task_plan->target.color.c_str());
  }

  std::optional<TaskPlan> selectTask(const my_robot_interfaces::msg::DetectedObjectArray & msg) const
  {
    for (const auto & grasp_candidate : msg.objects) {
      if (grasp_candidate.shape != "grasp") {
        continue;
      }

      for (const auto & target_candidate : msg.objects) {
        if (target_candidate.shape != "target") {
          continue;
        }
        if (target_candidate.color != grasp_candidate.color) {
          continue;
        }
        return TaskPlan{grasp_candidate, target_candidate};
      }
    }
    return std::nullopt;
  }

  void buildSequence(const TaskPlan & task_plan)
  {
    steps_.clear();

    const auto & grasp = task_plan.grasp;
    const auto & target = task_plan.target;

    const double grasp_z = grasp.height + grasp_surface_offset_;
    const double pregrasp_z = grasp.height + pregrasp_height_;
    const double lifted_z = grasp.height + lift_height_;
    const double preplace_z = target.height + preplace_clearance_;
    const double place_z = target.height + place_surface_offset_;

    steps_.push_back(makeGripperStep(true, gripper_wait_sec_, "open gripper"));
    steps_.push_back(makePoseStep(grasp.top_center.x, grasp.top_center.y, pregrasp_z, false, pose_wait_sec_, "move to pregrasp"));
    steps_.push_back(makePoseStep(grasp.top_center.x, grasp.top_center.y, grasp_z, true, cartesian_wait_sec_, "descend to grasp"));
    steps_.push_back(makeGripperStep(false, gripper_wait_sec_, "close gripper"));
    steps_.push_back(makePoseStep(grasp.top_center.x, grasp.top_center.y, lifted_z, true, cartesian_wait_sec_, "lift object"));
    steps_.push_back(makePoseStep(target.top_center.x, target.top_center.y, preplace_z, false, pose_wait_sec_, "move to preplace"));
    steps_.push_back(makePoseStep(target.top_center.x, target.top_center.y, place_z, true, cartesian_wait_sec_, "descend to place"));
    steps_.push_back(makeGripperStep(true, gripper_wait_sec_, "open gripper to release"));
    steps_.push_back(makePoseStep(target.top_center.x, target.top_center.y, preplace_z, true, cartesian_wait_sec_, "retreat from target"));
    steps_.push_back(makePoseStep(end_x_, end_y_, retreat_height_, false, pose_wait_sec_, "move to end pose"));
  }

  Step makePoseStep(
    double x, double y, double z, bool cartesian_path, double wait_seconds, const std::string & description) const
  {
    Step step;
    step.type = Step::Type::kPose;
    step.pose.x = x;
    step.pose.y = y;
    step.pose.z = z;
    step.pose.roll = roll_;
    step.pose.pitch = pitch_;
    step.pose.yaw = yaw_;
    step.pose.cartesian_path = cartesian_path;
    step.wait_seconds = wait_seconds;
    step.description = description;
    return step;
  }

  Step makeGripperStep(bool open_gripper, double wait_seconds, const std::string & description) const
  {
    Step step;
    step.type = Step::Type::kGripper;
    step.open_gripper = open_gripper;
    step.wait_seconds = wait_seconds;
    step.description = description;
    return step;
  }

  void runNextStep()
  {
    if (!sequence_started_) {
      return;
    }

    if (now() < next_step_ready_time_) {
      return;
    }

    if (current_step_index_ >= steps_.size()) {
      RCLCPP_INFO(get_logger(), "Sorting sequence completed");
      sequence_started_ = false;
      step_timer_->cancel();
      return;
    }

    const Step & step = steps_[current_step_index_];
    if (step.type == Step::Type::kPose) {
      pose_pub_->publish(step.pose);
      RCLCPP_INFO(
        get_logger(),
        "Step %zu/%zu: %s -> pose(%.3f, %.3f, %.3f) cartesian=%s",
        current_step_index_ + 1, steps_.size(), step.description.c_str(),
        step.pose.x, step.pose.y, step.pose.z,
        step.pose.cartesian_path ? "true" : "false");
    } else {
      example_interfaces::msg::Bool command;
      command.data = step.open_gripper;
      gripper_pub_->publish(command);
      RCLCPP_INFO(
        get_logger(),
        "Step %zu/%zu: %s -> open_gripper=%s",
        current_step_index_ + 1, steps_.size(), step.description.c_str(),
        step.open_gripper ? "true" : "false");
    }

    next_step_ready_time_ = now() + rclcpp::Duration::from_seconds(step.wait_seconds);
    ++current_step_index_;
  }

  rclcpp::Publisher<my_robot_interfaces::msg::PoseCommand>::SharedPtr pose_pub_;
  rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr gripper_pub_;
  rclcpp::Subscription<my_robot_interfaces::msg::DetectedObjectArray>::SharedPtr object_sub_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  my_robot_interfaces::msg::DetectedObjectArray::SharedPtr latest_objects_;
  std::vector<Step> steps_;
  bool sequence_started_{false};
  std::size_t current_step_index_{0};
  rclcpp::Time next_step_ready_time_{0, 0, RCL_ROS_TIME};
  std::string stable_signature_;
  int stable_hits_{0};

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
  bool wait_for_stable_detections_;
  int stable_detection_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorSortingNode>());
  rclcpp::shutdown();
  return 0;
}
