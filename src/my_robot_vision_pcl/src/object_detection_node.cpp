#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "my_robot_interfaces/msg/detected_object_pcl.hpp"
#include "my_robot_interfaces/msg/detected_object_pcl_array.hpp"

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
      "/vision/detected_objects");

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

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    publisher_ =
      create_publisher<
      my_robot_interfaces::msg::DetectedObjectPCLArray>(
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
  }

private:
  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(
      *msg,
      *input);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    const auto clusters =
      detector_->detect(
        input,
        object_cloud);

    my_robot_interfaces::msg::DetectedObjectPCLArray result;

    result.header = msg->header;

    result.objects.reserve(clusters.size());

    for (std::size_t i = 0; i < clusters.size(); ++i)
    {
      const auto & cluster = clusters[i];

      my_robot_interfaces::msg::DetectedObjectPCL object;

      object.id =
        static_cast<uint32_t>(i);

      // 颜色暂时不处理
      object.color = "";

      object.center.x = cluster.center_x;
      object.center.y = cluster.center_y;
      object.center.z = cluster.center_z;

      object.dimensions.x = cluster.size_x;
      object.dimensions.y = cluster.size_y;
      object.dimensions.z = cluster.size_z;

      result.objects.push_back(object);
    }

    publisher_->publish(result);
  }

  std::string input_topic_;
  std::string output_topic_;

  std::unique_ptr<ObjectDetector> detector_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<
    my_robot_interfaces::msg::DetectedObjectPCLArray>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<ObjectDetectionNode>());

  rclcpp::shutdown();

  return 0;
}