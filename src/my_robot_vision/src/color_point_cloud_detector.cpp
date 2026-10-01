#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <my_robot_interfaces/msg/detected_object_array.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "detection_stabilizer.hpp"
#include "point_cloud_processor.hpp"


class ColorPointCloudDetectorNode : public rclcpp::Node
{
public:
  ColorPointCloudDetectorNode()
  : Node("color_point_cloud_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),

    point_cloud_processor_(
      declare_parameter<double>("min_x", 0.25),
      declare_parameter<double>("max_x", 1.00),
      declare_parameter<double>("min_y", -1.00),
      declare_parameter<double>("max_y", 1.00),
      declare_parameter<double>("min_z", 0.01),
      declare_parameter<double>("max_z", 1.00),
      declare_parameter<double>("grid_resolution", 0.01),
      static_cast<std::size_t>(
        declare_parameter<int>("min_cluster_points", 120)),
      declare_parameter<double>("top_layer_thickness", 0.012),
      declare_parameter<double>(
        "height_category_threshold", 0.15),
      static_cast<std::size_t>(
        declare_parameter<int>("min_top_points", 40))),

    stabilizer_(
      declare_parameter<double>(
        "required_stable_time", 0.0))
  {
    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()));

    point_cloud_topic_ =
      declare_parameter<std::string>(
        "point_cloud_topic",
        "/camera_link/points");

    target_frame_ =
      declare_parameter<std::string>(
        "target_frame",
        "base_link");

    point_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &ColorPointCloudDetectorNode::pointCloudCallback,
          this,
          std::placeholders::_1));

    objects_pub_ =
      this->create_publisher<
        my_robot_interfaces::msg::DetectedObjectArray>(
        "/detected_objects",
        10);

    marker_pub_ =
      this->create_publisher<
        visualization_msgs::msg::MarkerArray>(
        "/detected_object_markers",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on %s and publishing detected object array",
      point_cloud_topic_.c_str());
  }

private:
  // ============================================================
  // Point cloud callback
  // ============================================================

  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->data.empty())
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Received empty point cloud");

      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;

    try
    {
      transform_stamped =
        tf_buffer_.lookupTransform(
          target_frame_,
          msg->header.frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "TF lookup to %s failed: %s",
        target_frame_.c_str(),
        ex.what());

      return;
    }

    tf2::Transform tf_sensor_to_target;

    tf2::fromMsg(
      transform_stamped.transform,
      tf_sensor_to_target);

    // ----------------------------------------------------------
    // 1. 点云处理
    // ----------------------------------------------------------

    std::vector<
      my_robot_interfaces::msg::DetectedObject>
      objects;

    objects =
      point_cloud_processor_.process(
        *msg,
        tf_sensor_to_target);

    // ----------------------------------------------------------
    // 2. 排序
    // ----------------------------------------------------------

    std::sort(
      objects.begin(),
      objects.end(),
      [](const auto &lhs, const auto &rhs)
      {
        if (lhs.top_center.x == rhs.top_center.x)
        {
          return lhs.top_center.y < rhs.top_center.y;
        }

        return lhs.top_center.x < rhs.top_center.x;
      });

    // ----------------------------------------------------------
    // 3. 生成对象 ID
    // ----------------------------------------------------------

    for (std::size_t i = 0; i < objects.size(); ++i)
    {
      objects[i].id =
        "obj_" + std::to_string(i);
    }

    // ----------------------------------------------------------
    // 4. 检测结果稳定性
    // ----------------------------------------------------------

    if (!stabilizer_.update(
        objects,
        this->now()))
    {
      return;
    }

    // ----------------------------------------------------------
    // 5. 发布稳定检测结果
    // ----------------------------------------------------------

    my_robot_interfaces::msg::DetectedObjectArray array_msg;

    array_msg.header = msg->header;
    array_msg.header.frame_id = target_frame_;
    array_msg.objects =
      stabilizer_.getStableObjects();

    objects_pub_->publish(array_msg);

    publishMarkers(array_msg);
  }

  // ============================================================
  // Visualization
  // ============================================================

  void publishMarkers(
    const my_robot_interfaces::msg::DetectedObjectArray &array_msg)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;

    delete_marker.header = array_msg.header;
    delete_marker.ns = "detected_objects";
    delete_marker.action =
      visualization_msgs::msg::Marker::DELETEALL;

    marker_array.markers.push_back(delete_marker);

    for (std::size_t i = 0;
         i < array_msg.objects.size();
         ++i)
    {
      const auto &object =
        array_msg.objects[i];

      // --------------------------------------------------------
      // Center sphere
      // --------------------------------------------------------

      visualization_msgs::msg::Marker sphere;

      sphere.header = array_msg.header;
      sphere.ns = "detected_objects_center";
      sphere.id = static_cast<int>(i);

      sphere.type =
        visualization_msgs::msg::Marker::SPHERE;

      sphere.action =
        visualization_msgs::msg::Marker::ADD;

      sphere.pose.position =
        object.top_center;

      sphere.pose.orientation.w = 1.0;

      sphere.scale.x = 0.04;
      sphere.scale.y = 0.04;
      sphere.scale.z = 0.04;

      sphere.color.a = 1.0;

      if (object.color == "red")
      {
        sphere.color.r = 1.0;
        sphere.color.g = 0.2;
        sphere.color.b = 0.2;
      }
      else if (object.color == "blue")
      {
        sphere.color.r = 0.2;
        sphere.color.g = 0.4;
        sphere.color.b = 1.0;
      }
      else
      {
        sphere.color.r = 0.8;
        sphere.color.g = 0.8;
        sphere.color.b = 0.8;
      }

      sphere.lifetime =
        rclcpp::Duration::from_seconds(0.2);

      marker_array.markers.push_back(sphere);

      // --------------------------------------------------------
      // Text
      // --------------------------------------------------------

      visualization_msgs::msg::Marker text;

      text.header = array_msg.header;
      text.ns = "detected_objects_text";
      text.id =
        static_cast<int>(i + 1000);

      text.type =
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

      text.action =
        visualization_msgs::msg::Marker::ADD;

      text.pose.position =
        object.top_center;

      text.pose.position.z += 0.06;

      text.pose.orientation.w = 1.0;

      text.scale.z = 0.035;

      text.color.a = 1.0;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;

      text.text =
        object.id + " " +
        object.color + " " +
        object.shape;

      text.lifetime =
        rclcpp::Duration::from_seconds(0.2);

      marker_array.markers.push_back(text);
    }

    marker_pub_->publish(marker_array);
  }

private:
  // ============================================================
  // TF
  // ============================================================

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ============================================================
  // Processing modules
  // ============================================================

  PointCloudProcessor point_cloud_processor_;
  DetectionStabilizer stabilizer_;

  // ============================================================
  // ROS interfaces
  // ============================================================

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;

  rclcpp::Publisher<
    my_robot_interfaces::msg::DetectedObjectArray>::SharedPtr
    objects_pub_;

  rclcpp::Publisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_;

  // ============================================================
  // Parameters
  // ============================================================

  std::string point_cloud_topic_;
  std::string target_frame_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<ColorPointCloudDetectorNode>());

  rclcpp::shutdown();

  return 0;
}