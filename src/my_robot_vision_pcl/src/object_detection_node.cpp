#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "my_robot_vision_pcl/object_detector.hpp"

class ObjectDetectionNode : public rclcpp::Node
{
public:
  ObjectDetectionNode()
  : Node("object_detection")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic",
      "/vision/filtered_points");

    output_topic_ = declare_parameter<std::string>(
      "output_topic",
      "/vision/object_points");

    double plane_distance_threshold =
      declare_parameter<double>(
      "plane_distance_threshold",
      0.015);

    double cluster_tolerance =
      declare_parameter<double>(
      "cluster_tolerance",
      0.03);

    int min_cluster_size =
      declare_parameter<int>(
      "min_cluster_size",
      30);

    int max_cluster_size =
      declare_parameter<int>(
      "max_cluster_size",
      5000);

    detector_ = std::make_unique<ObjectDetector>(
      plane_distance_threshold,
      cluster_tolerance,
      min_cluster_size,
      max_cluster_size);

    auto qos = rclcpp::QoS(
      rclcpp::KeepLast(10));

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
        &ObjectDetectionNode::pointCloudCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Object detection started.");

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
      "Plane threshold: %.3f m",
      plane_distance_threshold);

    RCLCPP_INFO(
      get_logger(),
      "Cluster tolerance: %.3f m",
      cluster_tolerance);

    RCLCPP_INFO(
      get_logger(),
      "Cluster size: %d ~ %d points",
      min_cluster_size,
      max_cluster_size);
  }

private:
  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(*msg, *input);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    const auto clusters =
      detector_->detect(
      input,
      object_cloud);

    // 发布去掉平面后的点云
    sensor_msgs::msg::PointCloud2 output_msg;

    pcl::toROSMsg(
      *object_cloud,
      output_msg);

    output_msg.header = msg->header;

    publisher_->publish(output_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Input: %zu points, object cloud: %zu points, clusters: %zu",
      input->points.size(),
      object_cloud->points.size(),
      clusters.size());

  }

  std::string input_topic_;
  std::string output_topic_;

  std::unique_ptr<ObjectDetector> detector_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<ObjectDetectionNode>());

  rclcpp::shutdown();

  return 0;
}