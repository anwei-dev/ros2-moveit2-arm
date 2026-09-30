#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "my_robot_vision_pcl/point_cloud_processor.hpp"

class PointCloudProcessorNode : public rclcpp::Node
{
public:
  PointCloudProcessorNode()
  : Node("point_cloud_processor")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic",
      "/camera_link/points");

    output_topic_ = declare_parameter<std::string>(
      "output_topic",
      "/vision/filtered_points");

    target_frame_ = declare_parameter<std::string>(
      "target_frame",
      "base_link");

    processor_ = std::make_unique<PointCloudProcessor>(
       0.25, 1.0,   // X in base_link
       -1.0, 1.0,    // Y in base_link
       -0.1, 2.0,    // Z in base_link
       0.01);       // voxel size

    // 摄像头使用 RELIABLE，因此这里也使用 RELIABLE
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      qos);

    subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(
        &PointCloudProcessorNode::pointCloudCallback,
        this,
        std::placeholders::_1));

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(
      this->get_clock());

    tf_listener_ =
      std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_);

    RCLCPP_INFO(
      get_logger(),
      "Point cloud processor started.");

    RCLCPP_INFO(
      get_logger(),
      "Input: %s",
      input_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Output: %s",
      output_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Target frame: %s",
      target_frame_.c_str());
  }

private:
  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 transformed_cloud;

    try
    {
      // 将当前时刻的点云从 camera_link_optical
      // 转换到 base_link
      auto transform = tf_buffer_->lookupTransform(
        target_frame_,
        msg->header.frame_id,
        msg->header.stamp,
        tf2::durationFromSec(0.1));

      tf2::doTransform(
        *msg,
        transformed_cloud,
        transform);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Point cloud transform failed: %s",
        ex.what());

      return;
    }

    // 此时 transformed_cloud 已经是 base_link 坐标系
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(
      transformed_cloud,
      *input);

    // 在 base_link 坐标系中进行 PCL 处理
    auto output = processor_->process(input);

    sensor_msgs::msg::PointCloud2 output_msg;

    pcl::toROSMsg(
      *output,
      output_msg);

    // PCL 处理后的结果保持 base_link 坐标系
    output_msg.header = transformed_cloud.header;

    publisher_->publish(output_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;

  std::unique_ptr<PointCloudProcessor> processor_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<PointCloudProcessorNode>());

  rclcpp::shutdown();

  return 0;
}