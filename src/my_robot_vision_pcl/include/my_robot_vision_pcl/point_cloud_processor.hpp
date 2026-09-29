#ifndef MY_ROBOT_VISION_PCL__POINT_CLOUD_PROCESSOR_HPP_
#define MY_ROBOT_VISION_PCL__POINT_CLOUD_PROCESSOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudProcessor
{
public:
  PointCloudProcessor(
    double min_x,
    double max_x,
    double min_y,
    double max_y,
    double min_z,
    double max_z,
    double voxel_size);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr process(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input);

private:
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
  double voxel_size_;
};

#endif