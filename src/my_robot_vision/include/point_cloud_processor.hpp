#ifndef COLOR_POINT_CLOUD_DETECTOR__POINT_CLOUD_PROCESSOR_HPP_
#define COLOR_POINT_CLOUD_DETECTOR__POINT_CLOUD_PROCESSOR_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <my_robot_interfaces/msg/detected_object.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "color_classifier.hpp"

class PointCloudProcessor
{
public:
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

  struct Cluster
  {
    std::vector<PointSample> points;
  };

  PointCloudProcessor(
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
    std::size_t min_top_points);

  std::vector<my_robot_interfaces::msg::DetectedObject>
  process(
    const sensor_msgs::msg::PointCloud2 &msg,
    const tf2::Transform &tf_sensor_to_target);

private:
  using CellMap =
    std::unordered_map<
      CellIndex,
      Cluster,
      CellIndexHash>;

  void collectPoints(
    const sensor_msgs::msg::PointCloud2 &msg,
    const tf2::Transform &tf_sensor_to_target,
    CellMap &cells);

  std::vector<Cluster> buildClusters(
    const CellMap &cells) const;

  bool buildDetectedObject(
    const std::vector<PointSample> &points,
    my_robot_interfaces::msg::DetectedObject &object) const;

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

  ColorClassifier color_classifier_;
};

#endif