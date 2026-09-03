#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

class MyTemplateNode : public rclcpp::Node
{
public:
  MyTemplateNode()
  : Node("my_template_node")
  {
    // 声明参数（示例）
    this->declare_parameter("example_param", "default_value");
    
    // 创建定时器（示例：每秒打印一次）
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MyTemplateNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "节点已启动");
  }

private:
  void timerCallback()
  {
    // 此处放置周期性执行的任务
    RCLCPP_DEBUG(this->get_logger(), "定时器触发");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyTemplateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}