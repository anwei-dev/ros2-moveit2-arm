#ifndef MY_ROBOT_VISION_PCL__OBJECT_DETECTOR_HPP_
#define MY_ROBOT_VISION_PCL__OBJECT_DETECTOR_HPP_

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct ClusterInfo
{
  std::size_t point_count;

  double center_x;
  double center_y;
  double center_z;

  double size_x;
  double size_y;
  double size_z;
};

class ObjectDetector
{
public:
  ObjectDetector(
    double plane_distance_threshold,
    double cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size);

  std::vector<ClusterInfo> detect(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_plane_cloud);

private:
  double plane_distance_threshold_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
};

#endif