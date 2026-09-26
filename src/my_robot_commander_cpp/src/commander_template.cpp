#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <my_robot_interfaces/action/move_to_pose.hpp>

#include <cmath>
#include <limits>

#include <thread>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using JointState = sensor_msgs::msg::JointState;
using StdFloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::placeholders;

using MoveToPose = my_robot_interfaces::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_effort_pub_ = node_->create_publisher<StdFloatArray>(
            "/gripper_controller/commands", 10);

        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper", 10, std::bind(&Commander::openGripperCallback, this, _1));

        joint_state_sub_ = node_->create_subscription<JointState>(
            "/joint_states", 10, std::bind(&Commander::jointStateCallback, this, _1));

        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));

        // pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
        //     "pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));
        // 改成 action
        move_to_pose_action_server_ = rclcpp_action::create_server<MoveToPose>(
            node_,
            "move_to_pose",
            std::bind(&Commander::handleGoal, this, _1, _2),
            std::bind(&Commander::handleCancel, this, _1),
            std::bind(&Commander::handleAccepted, this, _1));

        close_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Commander::closeLoopStep, this));
        close_timer_->cancel();
    }

    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    bool goToPoseTarget(
        double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path = false)
    {
        // 传入笛卡尔坐标和欧拉角参数
        tf2::Quaternion q;          // 创建保存欧拉角的变量q
        q.setRPY(roll, pitch, yaw); // 使用tf2::Quaternion类的成员函数setRPY获取欧拉角参数并赋值给变量q
        q = q.normalize();          // 归一化（取近似值）

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z; // 笛卡尔坐标的赋值
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW(); // 完成欧拉角转换后的四元数的赋值

        arm_->setStartStateToCurrentState();

        if (!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);
            return planAndExecute(arm_);
        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;
            double fraction = arm_->computeCartesianPath(
                waypoints,
                0.01,
                0.0,
                trajectory,
                false,
                &error_code);

            if (fraction >= 0.99)
            {
                auto execution_result = arm_->execute(trajectory);

                return execution_result ==
                       moveit::core::MoveItErrorCode::SUCCESS;
            }

            return false;
        }
    }

    void openGripper()
    {
        stopClosingLoop();
        publishGripperEffort(-open_effort_, open_effort_);
    }

    void closeGripper()
    {
        closing_active_ = true;
        stall_count_ = 0;
        last_left_position_ = std::numeric_limits<double>::quiet_NaN();
        last_right_position_ = std::numeric_limits<double>::quiet_NaN();
        close_timer_->reset();
        publishGripperEffort(close_effort_, -close_effort_);
    }

private:
    bool planAndExecute(
        const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;

        auto planning_result = interface->plan(plan);

        if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MoveIt2 planning failed, error_code=%d",
                planning_result.val);

            return false;
        }

        auto execution_result = interface->execute(plan);
        if (execution_result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MoveIt2 execution failed, error_code=%d",
                execution_result.val);

            return false;
        }

        return true;
    }

    void publishGripperEffort(double left_effort, double right_effort)
    {
        StdFloatArray command;
        command.data = {left_effort, right_effort};
        gripper_effort_pub_->publish(command);
    }

    void publishGripperStop()
    {
        publishGripperEffort(0.0, 0.0);
    }

    void stopClosingLoop()
    {
        closing_active_ = false;
        stall_count_ = 0;
        close_timer_->cancel();
        publishGripperStop();
    }

    void jointStateCallback(const JointState &msg)
    {
        const auto left_index = findJointIndex(msg, "gripper_left_finger_joint");
        const auto right_index = findJointIndex(msg, "gripper_right_finger_joint");
        if (left_index < 0 || right_index < 0)
        {
            return;
        }

        if (msg.position.size() <= static_cast<size_t>(left_index) ||
            msg.position.size() <= static_cast<size_t>(right_index))
        {
            return;
        }

        current_left_position_ = msg.position[static_cast<size_t>(left_index)];
        current_right_position_ = msg.position[static_cast<size_t>(right_index)];
        have_gripper_joint_state_ = true;
        last_joint_state_time_ = node_->now();
    }

    int findJointIndex(const JointState &msg, const std::string &joint_name) const
    {
        for (size_t i = 0; i < msg.name.size(); ++i)
        {
            if (msg.name[i] == joint_name)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    void closeLoopStep()
    {
        if (!closing_active_)
        {
            return;
        }

        if (!have_gripper_joint_state_)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "Waiting for /joint_states before closing gripper");
            publishGripperEffort(close_effort_, -close_effort_);
            return;
        }

        if ((node_->now() - last_joint_state_time_).seconds() > joint_state_timeout_sec_)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "Gripper close loop is waiting for fresh /joint_states");
            publishGripperEffort(close_effort_, -close_effort_);
            return;
        }

        if (current_left_position_ >= close_goal_left_ &&
            current_right_position_ <= close_goal_right_)
        {
            RCLCPP_INFO(node_->get_logger(), "Gripper reached close goal.");
            stopClosingLoop();
            return;
        }

        if (!std::isnan(last_left_position_) && !std::isnan(last_right_position_))
        {
            const double left_delta = std::abs(current_left_position_ - last_left_position_);
            const double right_delta = std::abs(current_right_position_ - last_right_position_);

            if (left_delta < stall_position_epsilon_ && right_delta < stall_position_epsilon_)
            {
                ++stall_count_;
            }
            else
            {
                stall_count_ = 0;
            }

            if (stall_count_ >= stall_cycle_threshold_)
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Gripper closing stopped by stall detection at left=%.4f right=%.4f",
                    current_left_position_, current_right_position_);
                stopClosingLoop();
                return;
            }
        }

        last_left_position_ = current_left_position_;
        last_right_position_ = current_right_position_;
        publishGripperEffort(close_effort_, -close_effort_);
    }

    void openGripperCallback(const Bool &msg)
    {
        if (msg.data)
        {
            openGripper();
        }
        else
        {
            closeGripper();
        }
    }

    void jointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;

        if (joints.size() == 6)
        {
            goToJointTarget(joints);
        }
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveToPose::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(
            node_->get_logger(),
            "Received MoveToPose goal: x=%.3f y=%.3f z=%.3f",
            goal->x,
            goal->y,
            goal->z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        (void)goal_handle;

        RCLCPP_INFO(
            node_->get_logger(),
            "Received request to cancel MoveToPose goal");

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        std::thread{
            std::bind(&Commander::executeMoveToPose, this, goal_handle)}
            .detach();
    }

    void executeMoveToPose(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<MoveToPose::Feedback>();
        auto result = std::make_shared<MoveToPose::Result>();

        feedback->state = "planning";
        feedback->progress = 0.0f;
        goal_handle->publish_feedback(feedback);

        const bool success = goToPoseTarget(
            goal->x,
            goal->y,
            goal->z,
            goal->roll,
            goal->pitch,
            goal->yaw,
            goal->cartesian_path);

        if (success)
        {
            feedback->state = "completed";
            feedback->progress = 1.0f;
            goal_handle->publish_feedback(feedback);

            result->success = true;
            result->error_code = 0;

            goal_handle->succeed(result);

            RCLCPP_INFO(
                node_->get_logger(),
                "MoveToPose succeeded");
        }
        else
        {
            feedback->state = "failed";
            feedback->progress = 0.0f;
            goal_handle->publish_feedback(feedback);

            result->success = false;
            result->error_code = 1;

            goal_handle->abort(result);

            RCLCPP_ERROR(
                node_->get_logger(),
                "MoveToPose failed");
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    rclcpp::Publisher<StdFloatArray>::SharedPtr gripper_effort_pub_;
    rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_action_server_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr close_timer_;
    const double close_effort_ = 4.0;
    const double open_effort_ = 5.0;
    const double close_goal_left_ = 0.025;
    const double close_goal_right_ = -0.025;
    const double stall_position_epsilon_ = 1e-4;
    const double joint_state_timeout_sec_ = 0.5;
    const int stall_cycle_threshold_ = 4;
    bool have_gripper_joint_state_ = false;
    bool closing_active_ = false;
    int stall_count_ = 0;
    double current_left_position_ = 0.0;
    double current_right_position_ = 0.0;
    double last_left_position_ = std::numeric_limits<double>::quiet_NaN();
    double last_right_position_ = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
