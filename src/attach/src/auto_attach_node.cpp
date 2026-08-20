#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <example_interfaces/msg/bool.hpp> // 新增：包含Bool消息类型
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_model_attachment_plugin_msgs/srv/attach.hpp>
#include <gazebo_model_attachment_plugin_msgs/srv/detach.hpp> // 新增：包含Detach服务类型
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using AttachSrv = gazebo_model_attachment_plugin_msgs::srv::Attach;
using DetachSrv = gazebo_model_attachment_plugin_msgs::srv::Detach; // 新增：定义Detach服务别名
using LinkStates = gazebo_msgs::msg::LinkStates;
using ModelStates = gazebo_msgs::msg::ModelStates;

class AutoAttachNode : public rclcpp::Node
{
public:
  AutoAttachNode()
  : Node("auto_attach_node")
  {
    // 创建attach client
    attach_client_ = create_client<AttachSrv>("/gazebo/attach");
    // 创建detach client
    detach_client_ = create_client<DetachSrv>("/gazebo/detach"); // 新增：创建detach客户端

    // 获取ros2系统中的 robot_link_name 作为基础link，若不存在，使用默认my_robot::attach_link
    robot_link_name_ = declare_parameter("robot_link_name", "my_robot::attach_link");
    // 被抓取links的列表
    candidate_models_ = declare_parameter<std::vector<std::string>>(
      "candidate_models", {"grasp_cube", "blue_cube"});
    // 横向搜索范围和纵向搜索范围
    max_distance_ = declare_parameter("max_distance", 0.12);
    max_height_error_ = declare_parameter("max_height_error", 0.15);

    // 发布订阅者读取/gazebo/model_states发布的动态模型（被抓取对象）的参数
    model_states_sub_ = create_subscription<ModelStates>(
      "/gazebo/model_states", 10,
      std::bind(&AutoAttachNode::onModelStates, this, std::placeholders::_1));
    // 和上面的相同，区别是这里主要目的是获取抓夹的位置，attach_link的位置
    link_states_sub_ = create_subscription<LinkStates>(
      "/gazebo/link_states", 10,
      std::bind(&AutoAttachNode::onLinkStates, this, std::placeholders::_1));

    // 新增：订阅 /open_gripper 话题
    gripper_sub_ = create_subscription<example_interfaces::msg::Bool>(
      "/open_gripper", 10,
      std::bind(&AutoAttachNode::onGripperCommand, this, std::placeholders::_1));

    std::ostringstream models_stream;
    for (size_t i = 0; i < candidate_models_.size(); ++i) {
      if (i != 0) {
        models_stream << ", ";
      }
      models_stream << candidate_models_[i];
    }
    RCLCPP_INFO(
      get_logger(), "Loaded candidate_models: [%s]", models_stream.str().c_str());
  }

private:
  void onModelStates(const ModelStates::SharedPtr msg)
  {
    latest_model_states_ = msg;
  }

  void onLinkStates(const LinkStates::SharedPtr msg)
  {
    latest_link_states_ = msg;
  }

  // 修改：处理来自 /open_gripper 话题的命令
  void onGripperCommand(const example_interfaces::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) { // 如果收到 data: false
      RCLCPP_INFO(this->get_logger(), "Gripper close command received. Attempting to attach.");
      performAttachmentLogic(); // 调用核心的抓取逻辑
    } else { // 如果收到 data: true
      RCLCPP_INFO(this->get_logger(), "Gripper open command received. Attempting to detach.");
      performDetachmentLogic(); // 调用核心的释放逻辑
    }
  }

  // 新增：执行释放逻辑
  void performDetachmentLogic()
  {
    // 如果没有正在抓取任何东西，则无需释放
    if (!attached_) {
      RCLCPP_INFO(this->get_logger(), "No object is currently attached. Nothing to detach.");
      return;
    }

    // 检查detach服务是否可用
    if (!detach_client_->wait_for_service(500ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "/gazebo/detach service unavailable");
      return;
    }

    // detach 请求需要与 attach 时的 joint 命名保持一致。
    auto request = std::make_shared<DetachSrv::Request>();
    request->joint_name = attached_model_ + "_joint";
    request->model_name_1 = "my_robot";
    request->model_name_2 = attached_model_;

    RCLCPP_INFO(
      get_logger(), "Auto detaching %s", attached_model_.c_str());

    // 发送异步detach请求
    detach_client_->async_send_request(
      request,
      [this](rclcpp::Client<DetachSrv>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(
            get_logger(), "Detach failed: %s", response->message.c_str());
          return;
        }

        // 重置状态
        attached_ = false;
        attached_model_.clear();
        RCLCPP_INFO(get_logger(), "Successfully detached object.");
      });
  }


  // 重命名：将原来的 tryAttach 逻辑移到这里
  void performAttachmentLogic()
  {
    // attached_是一个bool类型的成员变量，为真表示已经抓取
    // attach_pending_同样，为真表示正在抓取但未收到反馈
    if (attached_ || attach_pending_) {
      RCLCPP_DEBUG(this->get_logger(), "Attachment already in progress or completed. Skipping.");
      return;
    }
    // 检测这两个指针，如果都不为空才继续
    if (!latest_model_states_ || !latest_link_states_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for model/link states...");
      return;
    }
    // robot_link_name_是 my_robot::attach_link (默认值)
    // 寻找attach link的索引
    const int link_index = findLinkIndex(robot_link_name_);
    // 未找到，打印失败日志 这里我把 WARN 改成了 ERROR 更醒目
    if (link_index < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Robot link %s not found in /gazebo/link_states",
        robot_link_name_.c_str());
      return;
    }
    // 通过attach link的索引来在latest link states中来提取link pose
    // -> 用于通过指针访问对象成员
    const auto & link_pose = latest_link_states_->pose[static_cast<size_t>(link_index)];

    std::string target_model;
    // 声明了一个变量，并赋了一个极大的初始值。
    double best_distance = std::numeric_limits<double>::max();
    std::string closest_model;
    double closest_xy_distance = std::numeric_limits<double>::max();
    double closest_height_error = std::numeric_limits<double>::max();

    // 遍历 被抓取的方块们candidate models就是上面获取的所有那个，从一个topic订阅来的
    for (const auto & model_name : candidate_models_) {
      // 通过定义的函数获取对应model name的index
      const int model_index = findModelIndex(model_name);
      // 没获取到，跳过
      if (model_index < 0) {
        continue;
      }
      // 获取到了，提取目标的坐标，计算横向距离和高度差
      const auto & pose = latest_model_states_->pose[static_cast<size_t>(model_index)];
      const double dx = pose.position.x - link_pose.position.x;
      const double dy = pose.position.y - link_pose.position.y;
      const double dz = pose.position.z - link_pose.position.z;
      const double xy_distance = std::hypot(dx, dy);
      const double height_error = std::abs(dz);

      if (xy_distance < closest_xy_distance) {
        closest_xy_distance = xy_distance;
        closest_height_error = height_error;
        closest_model = model_name;
      }

      if (xy_distance > max_distance_ || height_error > max_height_error_) {
        continue;
      }
      // 如果xy distance更小，那就把他的值付给best distance
      if (xy_distance < best_distance) {
        best_distance = xy_distance;
        target_model = model_name;
      }
    }

    if (target_model.empty()) {
      if (!closest_model.empty()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "No attach target yet. Closest model=%s xy=%.3f z=%.3f thresholds=(%.3f, %.3f)",
          closest_model.c_str(), closest_xy_distance, closest_height_error,
          max_distance_, max_height_error_);
      }
      return;
    }

    if (!attach_client_->wait_for_service(500ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "/gazebo/attach service unavailable");
      return;
    }

    // 声明并配置传给attach服务的变量
    auto request = std::make_shared<AttachSrv::Request>();
    request->joint_name = target_model + "_joint";
    request->model_name_1 = "my_robot";
    request->link_name_1 = "attach_link";
    request->model_name_2 = target_model;
    request->link_name_2 = "link";


    RCLCPP_INFO(
      get_logger(), "Auto attaching %s at xy distance %.3f", target_model.c_str(), best_distance);

    attach_pending_ = true;
    attach_client_->async_send_request(
      request,
      [this, target_model](rclcpp::Client<AttachSrv>::SharedFuture future) {
        attach_pending_ = false;

        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(
            get_logger(), "Attach failed for %s: %s",
            target_model.c_str(), response->message.c_str());
          return;
        }

        attached_ = true;
        attached_model_ = target_model;
        RCLCPP_INFO(get_logger(), "Attached %s", attached_model_.c_str());
      });
  }

  // 一个用于通过模型名称获取传入被抓取物体在latest_model_states中的索引数字的函数
  int findModelIndex(const std::string & name) const
  {
    // 判断是否为空，为空返回-1代表失败
    if (!latest_model_states_) {
      return -1;
    }
    // 提取出所有对象的名称，所有被抓取模型的name
    const auto & names = latest_model_states_->name;
    // 在name中搜索，搜不到的话给默认值names.end() 
    const auto it = std::find(names.begin(), names.end(), name);
    // 判断搜索结果如，names.end()是空的，表示没找到目标
    if (it == names.end()) {
      return -1;
    }
    // 搜索成功，返回结果的索引（找到的结果到it之间有多少个元素）完成类型转换然后返回
    return static_cast<int>(std::distance(names.begin(), it));
  }

  // 获取连接在索引中的位置，主要用于找attach_link的索引
  int findLinkIndex(const std::string & name) const
  {
    if (!latest_link_states_) {
      return -1;
    }

    const auto & names = latest_link_states_->name;
    const auto it = std::find(names.begin(), names.end(), name);
    if (it == names.end()) {
      return -1;
    }
    return static_cast<int>(std::distance(names.begin(), it));
  }

  // ... (成员变量声明保持不变，移除了 timer_ 和 poll_period_sec_)

  rclcpp::Client<AttachSrv>::SharedPtr attach_client_;
  rclcpp::Client<DetachSrv>::SharedPtr detach_client_; // 新增：detach客户端
  rclcpp::Subscription<ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<LinkStates>::SharedPtr link_states_sub_;
  // 新增：订阅者
  rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr gripper_sub_;
  // 移除：rclcpp::TimerBase::SharedPtr timer_; 
  ModelStates::SharedPtr latest_model_states_;
  LinkStates::SharedPtr latest_link_states_;

  std::string robot_link_name_;
  std::vector<std::string> candidate_models_;
  std::string attached_model_;
  double max_distance_;
  double max_height_error_;
  // 移除：double poll_period_sec_;
  bool attached_ = false;
  bool attach_pending_ = false; // 保留，用于attach操作的pending状态
  // 移除单独的detach_pending_，因为detach逻辑通常很快，可以简化
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoAttachNode>());
  rclcpp::shutdown();
  return 0;
}
