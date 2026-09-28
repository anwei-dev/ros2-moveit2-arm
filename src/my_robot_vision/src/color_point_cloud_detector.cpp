#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <my_robot_interfaces/msg/detected_object.hpp>
#include <my_robot_interfaces/msg/detected_object_array.hpp>
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
#include <visualization_msgs/msg/marker_array.hpp>

class ColorPointCloudDetectorNode : public rclcpp::Node
{
public:
  ColorPointCloudDetectorNode()
      : Node("color_point_cloud_detector"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    tf_buffer_.setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface()));

    point_cloud_topic_ =
        declare_parameter<std::string>(
            "point_cloud_topic", "/camera_link/points");

    target_frame_ =
        declare_parameter<std::string>(
            "target_frame", "base_link");

    min_x_ = declare_parameter("min_x", 0.25);
    max_x_ = declare_parameter("max_x", 1.00);
    min_y_ = declare_parameter("min_y", -1.00);
    max_y_ = declare_parameter("max_y", 1.00);
    min_z_ = declare_parameter("min_z", 0.01);
    max_z_ = declare_parameter("max_z", 1.00);

    grid_resolution_ =
        declare_parameter("grid_resolution", 0.01);

    min_cluster_points_ =
        static_cast<std::size_t>(
            declare_parameter("min_cluster_points", 120));

    top_layer_thickness_ =
        declare_parameter("top_layer_thickness", 0.012);

    height_category_threshold_ =
        declare_parameter("height_category_threshold", 0.15);

    min_top_points_ =
        static_cast<std::size_t>(
            declare_parameter("min_top_points", 40));

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
            "/detected_objects", 10);

    marker_pub_ =
        this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
            "/detected_object_markers", 10);

    RCLCPP_INFO(
        this->get_logger(),
        "Listening on %s and publishing detected object array",
        point_cloud_topic_.c_str());
  }

private:
  enum class ColorLabel
  {
    kUnknown,
    kRed,
    kBlue
  };

  struct PointSample
  {
    double x;
    double y;
    double z;
    ColorLabel color;
  };

  struct CellIndex
  {
    int x;
    int y;

    bool operator==(const CellIndex &other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex &cell) const
    {
      const std::size_t hx = std::hash<int>{}(cell.x);
      const std::size_t hy = std::hash<int>{}(cell.y);
      return hx ^ (hy << 1);
    }
  };

  struct ClusterAccumulator
  {
    std::vector<PointSample> points;
  };

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
      transform_stamped = tf_buffer_.lookupTransform(
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

    std::unordered_map<
        CellIndex,
        ClusterAccumulator,
        CellIndexHash>
        cells;

    collectPoints(
        *msg,
        tf_sensor_to_target,
        cells);

    const auto clusters = buildClusters(cells);

    std::vector<
        my_robot_interfaces::msg::DetectedObject>
        objects;

    objects.reserve(clusters.size());

    for (const auto &cluster : clusters)
    {
      if (cluster.points.size() < min_cluster_points_)
      {
        continue;
      }

      my_robot_interfaces::msg::DetectedObject object;

      if (!buildDetectedObject(
              cluster.points,
              object))
      {
        continue;
      }

      objects.push_back(object);
    }

    // ------------------------------------------------------------
    // Sort objects
    // ------------------------------------------------------------

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

    // ------------------------------------------------------------
    // Generate object IDs
    // ------------------------------------------------------------

    for (std::size_t i = 0; i < objects.size(); ++i)
    {
      objects[i].id = "obj_" + std::to_string(i);
    }

    // ------------------------------------------------------------
    // Detection stability filtering
    // ------------------------------------------------------------

    // 第一次获得检测结果
    if (!has_stable_candidate_)
    {
      stable_candidate_objects_ = objects;
      stable_start_time_ = this->now();
      has_stable_candidate_ = true;
      is_detection_stable_ = false;

      RCLCPP_INFO(
          this->get_logger(),
          "New detection candidate received. "
          "Waiting for 3 seconds of stability.");

      return;
    }

    // ------------------------------------------------------------
    // 当前帧与稳定候选进行比较
    // ------------------------------------------------------------

    if (!objectsStable(
            stable_candidate_objects_,
            objects))
    {
      // 检测结果发生变化
      // 当前帧成为新的候选结果
      stable_candidate_objects_ = objects;

      // 重新开始计时
      stable_start_time_ = this->now();

      if (is_detection_stable_)
      {
        is_detection_stable_ = false;

        RCLCPP_INFO(
            this->get_logger(),
            "Detection became unstable. "
            "Publishing paused.");
      }
      else
      {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Detection unstable, restarting stability timer.");
      }

      return;
    }

    // ------------------------------------------------------------
    // 当前帧仍然稳定
    // ------------------------------------------------------------

    if (!is_detection_stable_)
    {
      const double stable_duration =
          (this->now() - stable_start_time_).seconds();

      if (stable_duration < kRequiredStableTime)
      {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Detection stable for %.1f / %.1f seconds.",
            stable_duration,
            kRequiredStableTime);

        return;
      }

      // 连续稳定达到 3 秒
      is_detection_stable_ = true;

      RCLCPP_INFO(
          this->get_logger(),
          "Detection stabilized for %.1f seconds. "
          "Start publishing.",
          stable_duration);
    }

    // ------------------------------------------------------------
    // 稳定状态：持续发布
    // ------------------------------------------------------------

    my_robot_interfaces::msg::DetectedObjectArray array_msg;

    array_msg.header = msg->header;
    array_msg.header.frame_id = target_frame_;
    array_msg.objects = stable_candidate_objects_;

    objects_pub_->publish(array_msg);
    publishMarkers(array_msg);
  }

  // ============================================================
  // Point collection
  // ============================================================

  void collectPoints(
      const sensor_msgs::msg::PointCloud2 &msg,
      const tf2::Transform &tf_sensor_to_target,
      std::unordered_map<
          CellIndex,
          ClusterAccumulator,
          CellIndexHash> &cells)
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(msg, "rgb");

    for (;
         iter_x != iter_x.end();
         ++iter_x,
         ++iter_y,
         ++iter_z,
         ++iter_rgb)
    {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      if (!std::isfinite(x) ||
          !std::isfinite(y) ||
          !std::isfinite(z))
      {
        continue;
      }

      const tf2::Vector3 point_target =
          tf_sensor_to_target *
          tf2::Vector3(x, y, z);

      const double px = point_target.x();
      const double py = point_target.y();
      const double pz = point_target.z();

      if (px < min_x_ ||
          px > max_x_ ||
          py < min_y_ ||
          py > max_y_ ||
          pz < min_z_ ||
          pz > max_z_)
      {
        continue;
      }

      const CellIndex cell{
          static_cast<int>(
              std::floor(px / grid_resolution_)),
          static_cast<int>(
              std::floor(py / grid_resolution_))};

      cells[cell].points.push_back(
          PointSample{
              px,
              py,
              pz,
              decodeColor(*iter_rgb)});
    }
  }

  // ============================================================
  // Build clusters
  // ============================================================

  std::vector<ClusterAccumulator> buildClusters(
      const std::unordered_map<
          CellIndex,
          ClusterAccumulator,
          CellIndexHash> &cells) const
  {
    std::vector<ClusterAccumulator> clusters;

    std::unordered_map<
        CellIndex,
        bool,
        CellIndexHash>
        visited;

    visited.reserve(cells.size());

    for (const auto &entry : cells)
    {
      const CellIndex &start = entry.first;

      if (visited[start])
      {
        continue;
      }

      ClusterAccumulator cluster;

      std::vector<CellIndex> stack{start};

      visited[start] = true;

      while (!stack.empty())
      {
        const CellIndex current = stack.back();
        stack.pop_back();

        const auto cell_it = cells.find(current);

        if (cell_it == cells.end())
        {
          continue;
        }

        const auto &cell_points =
            cell_it->second.points;

        cluster.points.insert(
            cluster.points.end(),
            cell_points.begin(),
            cell_points.end());

        for (int dx = -1; dx <= 1; ++dx)
        {
          for (int dy = -1; dy <= 1; ++dy)
          {
            if (dx == 0 && dy == 0)
            {
              continue;
            }

            const CellIndex neighbor{
                current.x + dx,
                current.y + dy};

            if (cells.find(neighbor) == cells.end() ||
                visited[neighbor])
            {
              continue;
            }

            visited[neighbor] = true;
            stack.push_back(neighbor);
          }
        }
      }

      clusters.push_back(std::move(cluster));
    }

    return clusters;
  }

  // ============================================================
  // Build detected object
  // ============================================================

  bool buildDetectedObject(
      const std::vector<PointSample> &points,
      my_robot_interfaces::msg::DetectedObject &object) const
  {
    double z_max =
        -std::numeric_limits<double>::infinity();

    std::array<std::size_t, 3> color_counts{0, 0, 0};

    for (const auto &point : points)
    {
      z_max = std::max(z_max, point.z);

      ++colorCountsRef(
          color_counts,
          point.color);
    }

    std::vector<const PointSample *> top_points;

    top_points.reserve(points.size());

    for (const auto &point : points)
    {
      if (point.z >= z_max - top_layer_thickness_)
      {
        top_points.push_back(&point);
      }
    }

    if (top_points.size() < min_top_points_)
    {
      return false;
    }

    geometry_msgs::msg::Point top_center;

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    double min_top_x =
        std::numeric_limits<double>::infinity();

    double max_top_x =
        -std::numeric_limits<double>::infinity();

    double min_top_y =
        std::numeric_limits<double>::infinity();

    double max_top_y =
        -std::numeric_limits<double>::infinity();

    for (const auto *point : top_points)
    {
      sum_x += point->x;
      sum_y += point->y;
      sum_z += point->z;

      min_top_x =
          std::min(min_top_x, point->x);

      max_top_x =
          std::max(max_top_x, point->x);

      min_top_y =
          std::min(min_top_y, point->y);

      max_top_y =
          std::max(max_top_y, point->y);
    }

    top_center.x =
        sum_x /
        static_cast<double>(top_points.size());

    top_center.y =
        sum_y /
        static_cast<double>(top_points.size());

    top_center.z =
        sum_z /
        static_cast<double>(top_points.size());

    const double span_x =
        max_top_x - min_top_x;

    const double diameter_x = span_x;

    const std::string color =
        colorToString(
            dominantColor(color_counts));

    const std::string shape =
        classifyCategory(z_max);

    object.id.clear();
    object.color = color;
    object.top_center = top_center;
    object.height = z_max;
    object.diameter_x = diameter_x;
    object.shape = shape;

    return true;
  }

  // ============================================================
  // Stability check
  // ============================================================

  bool objectsStable(
      const std::vector<
          my_robot_interfaces::msg::DetectedObject> &a,
      const std::vector<
          my_robot_interfaces::msg::DetectedObject> &b) const
  {
    // 对象数量变化
    if (a.size() != b.size())
    {
      return false;
    }

    for (std::size_t i = 0; i < a.size(); ++i)
    {
      const auto &obj_a = a[i];
      const auto &obj_b = b[i];

      // 颜色变化
      if (obj_a.color != obj_b.color)
      {
        return false;
      }

      // 类别变化
      if (obj_a.shape != obj_b.shape)
      {
        return false;
      }

      // X/Y/Z 位置变化超过 5 mm
      if (std::abs(
              obj_a.top_center.x -
              obj_b.top_center.x) >
          kPositionThreshold)
      {
        return false;
      }

      if (std::abs(
              obj_a.top_center.y -
              obj_b.top_center.y) >
          kPositionThreshold)
      {
        return false;
      }

      if (std::abs(
              obj_a.top_center.z -
              obj_b.top_center.z) >
          kPositionThreshold)
      {
        return false;
      }

      // 高度变化超过 5 mm
      if (std::abs(
              obj_a.height -
              obj_b.height) >
          kSizeThreshold)
      {
        return false;
      }

      // 直径变化超过 5 mm
      if (std::abs(
              obj_a.diameter_x -
              obj_b.diameter_x) >
          kSizeThreshold)
      {
        return false;
      }
    }

    return true;
  }

  // ============================================================
  // Classification
  // ============================================================

  std::string classifyCategory(double height) const
  {
    return height > height_category_threshold_
               ? "target"
               : "grasp";
  }

  // ============================================================
  // Color utilities
  // ============================================================

  static std::size_t &colorCountsRef(
      std::array<std::size_t, 3> &counts,
      ColorLabel color)
  {
    switch (color)
    {
    case ColorLabel::kRed:
      return counts[0];

    case ColorLabel::kBlue:
      return counts[1];

    case ColorLabel::kUnknown:
    default:
      return counts[2];
    }
  }

  static ColorLabel dominantColor(
      const std::array<std::size_t, 3> &counts)
  {
    if (counts[0] == 0 &&
        counts[1] == 0)
    {
      return ColorLabel::kUnknown;
    }

    return counts[0] >= counts[1]
               ? ColorLabel::kRed
               : ColorLabel::kBlue;
  }

  static std::string colorToString(
      ColorLabel color)
  {
    switch (color)
    {
    case ColorLabel::kRed:
      return "red";

    case ColorLabel::kBlue:
      return "blue";

    case ColorLabel::kUnknown:
    default:
      return "unknown";
    }
  }

  ColorLabel decodeColor(float rgb_float) const
  {
    std::uint32_t rgb_packed = 0;

    std::memcpy(
        &rgb_packed,
        &rgb_float,
        sizeof(float));

    const std::uint8_t r =
        static_cast<std::uint8_t>(
            (rgb_packed >> 16) & 0xFF);

    const std::uint8_t g =
        static_cast<std::uint8_t>(
            (rgb_packed >> 8) & 0xFF);

    const std::uint8_t b =
        static_cast<std::uint8_t>(
            rgb_packed & 0xFF);

    const std::uint8_t max_channel =
        std::max({r, g, b});

    if (max_channel < 50)
    {
      return ColorLabel::kUnknown;
    }

    if (r >= 90 &&
        r > static_cast<std::uint8_t>(g * 1.25) &&
        r > static_cast<std::uint8_t>(b * 1.25))
    {
      return ColorLabel::kRed;
    }

    if (b >= 90 &&
        b > static_cast<std::uint8_t>(r * 1.25) &&
        b > static_cast<std::uint8_t>(g * 1.10))
    {
      return ColorLabel::kBlue;
    }

    return ColorLabel::kUnknown;
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

      visualization_msgs::msg::Marker text;

      text.header = array_msg.header;
      text.ns = "detected_objects_text";
      text.id = static_cast<int>(i + 1000);
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

  // ============================================================
  // ROS interfaces
  // ============================================================

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<
      sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;

  rclcpp::Publisher<
      my_robot_interfaces::msg::DetectedObjectArray>::SharedPtr objects_pub_;

  rclcpp::Publisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ============================================================
  // Parameters
  // ============================================================

  std::string point_cloud_topic_;
  std::string target_frame_;

  double min_x_;
  double max_x_;

  double min_y_;
  double max_y_;

  double min_z_;
  double max_z_;

  double grid_resolution_;

  std::size_t min_cluster_points_;

  double top_layer_thickness_;

  double height_category_threshold_;

  std::size_t min_top_points_;

  // ============================================================
  // Detection stability
  // ============================================================

  static constexpr double kRequiredStableTime = 5.0; // 3 秒

  rclcpp::Time stable_start_time_;

  // 位置变化阈值：5 mm
  static constexpr double kPositionThreshold = 0.005;

  // 尺寸变化阈值：5 mm
  static constexpr double kSizeThreshold = 0.005;

  // 当前正在等待确认的稳定候选
  std::vector<
      my_robot_interfaces::msg::DetectedObject>
      stable_candidate_objects_;

  // 是否已经存在稳定候选
  bool has_stable_candidate_ = false;

  // 当前是否已经进入稳定发布状态
  bool is_detection_stable_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
      std::make_shared<ColorPointCloudDetectorNode>());

  rclcpp::shutdown();

  return 0;
}