#include "my_robot_vision_pcl/point_cloud_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

PointCloudProcessor::PointCloudProcessor(
  double min_x,
  double max_x,
  double min_y,
  double max_y,
  double min_z,
  double max_z,
  double voxel_size)
: min_x_(min_x),
  max_x_(max_x),
  min_y_(min_y),
  max_y_(max_y),
  min_z_(min_z),
  max_z_(max_z),
  voxel_size_(voxel_size)
{
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
PointCloudProcessor::process(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input)
{
  auto filtered =
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // 1. 移除 NaN 点
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *filtered, indices);

  // 2. VoxelGrid 降采样
  auto downsampled =
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud(filtered);
  voxel_filter.setLeafSize(
    static_cast<float>(voxel_size_),
    static_cast<float>(voxel_size_),
    static_cast<float>(voxel_size_));

  voxel_filter.filter(*downsampled);

  // 3. X 方向 ROI
  auto x_filtered =
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  pass_x.setInputCloud(downsampled);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(
    static_cast<float>(min_x_),
    static_cast<float>(max_x_));
  pass_x.filter(*x_filtered);

  // 4. Y 方向 ROI
  auto y_filtered =
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PassThrough<pcl::PointXYZRGB> pass_y;
  pass_y.setInputCloud(x_filtered);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(
    static_cast<float>(min_y_),
    static_cast<float>(max_y_));
  pass_y.filter(*y_filtered);

  // 5. Z 方向 ROI
  auto output =
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PassThrough<pcl::PointXYZRGB> pass_z;
  pass_z.setInputCloud(y_filtered);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(
    static_cast<float>(min_z_),
    static_cast<float>(max_z_));
  pass_z.filter(*output);

  return output;
}