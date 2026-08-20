#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

class PointCloudBoxCenterNode : public rclcpp::Node
{
public:
  PointCloudBoxCenterNode()
  : Node("point_cloud_box_center"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()));

    point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera_link/points", rclcpp::SensorDataQoS(),
      std::bind(&PointCloudBoxCenterNode::pointCloudCallback, this, std::placeholders::_1));

    center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/detected_box_center", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/detected_box_marker", 10);

    RCLCPP_INFO(this->get_logger(), "Listening on /camera_link/points");
  }

private:
  struct BasePoint
  {
    double x;
    double y;
    double z;
  };

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->data.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Received empty point cloud");
      return;
    }

    if (msg->height == 0 || msg->width == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Invalid point cloud dimensions");
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        "base_link", msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "TF lookup to base_link failed: %s", ex.what());
      return;
    }

    tf2::Transform tf_camera_to_base;
    tf2::fromMsg(transform_stamped.transform, tf_camera_to_base);

    std::vector<BasePoint> roi_points;
    roi_points.reserve(static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height) / 8);
    double z_max = -std::numeric_limits<double>::infinity();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const tf2::Vector3 point_base = tf_camera_to_base * tf2::Vector3(x, y, z);
      const double bx = point_base.x();
      const double by = point_base.y();
      const double bz = point_base.z();

      // ROI in base_link frame. Keep only points in the tabletop work area.
      if (bx < 0.30 || bx > 1.00) {
        continue;
      }
      if (by < -0.35 || by > 0.35) {
        continue;
      }
      // Remove floor/table points and keep only the upper part of the box.
      if (bz < 0.03 || bz > 0.90) {
        continue;
      }

      roi_points.push_back(BasePoint{bx, by, bz});
      z_max = std::max(z_max, bz);
    }

    if (roi_points.size() < 50) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Too few ROI points to estimate target center");
      return;
    }

    constexpr double kTopLayerThickness = 0.015;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    std::size_t count = 0;

    for (const auto & point : roi_points) {
      if (point.z < z_max - kTopLayerThickness) {
        continue;
      }

      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
      ++count;
    }

    if (count < 20) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Too few top-layer points to estimate target center");
      return;
    }

    geometry_msgs::msg::PointStamped center_base;
    center_base.header.frame_id = "base_link";
    center_base.header.stamp = msg->header.stamp;
    center_base.point.x = sum_x / static_cast<double>(count);
    center_base.point.y = sum_y / static_cast<double>(count);
    center_base.point.z = sum_z / static_cast<double>(count);

    center_pub_->publish(center_base);
    publishMarker(center_base);
  }

  void publishMarker(const geometry_msgs::msg::PointStamped & center)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = center.header;
    marker.ns = "detected_box_center";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = center.point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_pub_->publish(marker);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudBoxCenterNode>());
  rclcpp::shutdown();
  return 0;
}
