#include "point_cloud_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

PointCloudProcessor::PointCloudProcessor(
    double min_x,
    double max_x,
    double min_y,
    double max_y,
    double min_z,
    double max_z,
    double grid_resolution,
    std::size_t min_cluster_points,
    double top_layer_thickness,
    double height_category_threshold,
    std::size_t min_top_points)
    : min_x_(min_x),
      max_x_(max_x),
      min_y_(min_y),
      max_y_(max_y),
      min_z_(min_z),
      max_z_(max_z),
      grid_resolution_(grid_resolution),
      min_cluster_points_(min_cluster_points),
      top_layer_thickness_(top_layer_thickness),
      height_category_threshold_(height_category_threshold),
      min_top_points_(min_top_points)
{
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
PointCloudProcessor::convertToPCL(
    const sensor_msgs::msg::PointCloud2 &msg,
    const tf2::Transform &tf_sensor_to_target) const
{
  auto cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

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

    pcl::PointXYZRGB point;

    point.x = static_cast<float>(px);
    point.y = static_cast<float>(py);
    point.z = static_cast<float>(pz);

    const std::uint32_t rgb =
        static_cast<std::uint32_t>(*iter_rgb);

    point.rgb =
        *reinterpret_cast<const float *>(&rgb);

    cloud->points.push_back(point);
  }

  cloud->width =
      static_cast<std::uint32_t>(cloud->points.size());

  cloud->height = 1;

  cloud->is_dense = true;

  return cloud;
}

std::vector<my_robot_interfaces::msg::DetectedObject>
PointCloudProcessor::process(
    const sensor_msgs::msg::PointCloud2 &msg,
    const tf2::Transform &tf_sensor_to_target)
{
  const auto cloud =
      convertToPCL(
          msg,
          tf_sensor_to_target);

  (void)cloud;

  CellMap cells;

  collectPoints(
      msg,
      tf_sensor_to_target,
      cells);

  const auto clusters = buildClusters(cells);

  std::vector<my_robot_interfaces::msg::DetectedObject>
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

  for (std::size_t i = 0; i < objects.size(); ++i)
  {
    objects[i].id =
        "obj_" + std::to_string(i);
  }

  return objects;
}

void PointCloudProcessor::collectPoints(
    const sensor_msgs::msg::PointCloud2 &msg,
    const tf2::Transform &tf_sensor_to_target,
    CellMap &cells)
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
            color_classifier_.classify(*iter_rgb)});
  }
}

std::vector<PointCloudProcessor::Cluster>
PointCloudProcessor::buildClusters(
    const CellMap &cells) const
{
  std::vector<Cluster> clusters;

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

    Cluster cluster;

    std::vector<CellIndex> stack{start};

    visited[start] = true;

    while (!stack.empty())
    {
      const CellIndex current =
          stack.back();

      stack.pop_back();

      const auto cell_it =
          cells.find(current);

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

bool PointCloudProcessor::buildDetectedObject(
    const std::vector<PointSample> &points,
    my_robot_interfaces::msg::DetectedObject &object) const
{
  double z_max =
      -std::numeric_limits<double>::infinity();

  std::size_t red_count = 0;
  std::size_t blue_count = 0;
  std::size_t unknown_count = 0;

  for (const auto &point : points)
  {
    z_max = std::max(z_max, point.z);

    switch (point.color)
    {
    case ColorLabel::kRed:
      ++red_count;
      break;

    case ColorLabel::kBlue:
      ++blue_count;
      break;

    case ColorLabel::kUnknown:
    default:
      ++unknown_count;
      break;
    }
  }

  std::vector<const PointSample *> top_points;

  top_points.reserve(points.size());

  for (const auto &point : points)
  {
    if (point.z >=
        z_max - top_layer_thickness_)
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

  const double diameter_x =
      max_top_x - min_top_x;

  const ColorLabel dominant_color =
      ColorClassifier::dominantColor(
          red_count,
          blue_count,
          unknown_count);

  const std::string color =
      ColorClassifier::toString(dominant_color);

  const std::string shape =
      z_max > height_category_threshold_
          ? "target"
          : "grasp";

  object.id.clear();
  object.color = color;
  object.top_center = top_center;
  object.height = z_max;
  object.diameter_x = diameter_x;
  object.shape = shape;

  return true;
}