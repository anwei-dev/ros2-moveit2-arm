#include "my_robot_vision_pcl/object_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

ObjectDetector::ObjectDetector(
  double plane_distance_threshold,
  double cluster_tolerance,
  int min_cluster_size,
  int max_cluster_size)
: plane_distance_threshold_(plane_distance_threshold),
  cluster_tolerance_(cluster_tolerance),
  min_cluster_size_(min_cluster_size),
  max_cluster_size_(max_cluster_size)
{
}

std::vector<ClusterInfo> ObjectDetector::detect(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_plane_cloud)
{
  std::vector<ClusterInfo> results;

  if (!input || input->empty())
  {
    non_plane_cloud->clear();
    return results;
  }

  // ============================================================
  // 1. RANSAC 平面分割
  // ============================================================

  pcl::PointIndices::Ptr plane_inliers(
    new pcl::PointIndices);

  pcl::ModelCoefficients::Ptr plane_coefficients(
    new pcl::ModelCoefficients);

  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;

  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(
    plane_distance_threshold_);

  segmentation.setInputCloud(input);

  segmentation.segment(
    *plane_inliers,
    *plane_coefficients);

  // 没找到平面时，直接使用原始输入继续聚类
  if (plane_inliers->indices.empty())
  {
    *non_plane_cloud = *input;
  }
  else
  {
    // ==========================================================
    // 2. 去掉平面
    // ==========================================================

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    extract.setInputCloud(input);
    extract.setIndices(plane_inliers);
    extract.setNegative(true);

    extract.filter(*non_plane_cloud);
  }

  if (non_plane_cloud->empty())
  {
    return results;
  }

  // ============================================================
  // 3. 建立 KD-Tree
  // ============================================================

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZRGB>);

  tree->setInputCloud(non_plane_cloud);

  // ============================================================
  // 4. Euclidean Cluster
  // ============================================================

  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;

  clustering.setClusterTolerance(
    cluster_tolerance_);

  clustering.setMinClusterSize(
    min_cluster_size_);

  clustering.setMaxClusterSize(
    max_cluster_size_);

  clustering.setSearchMethod(tree);
  clustering.setInputCloud(non_plane_cloud);

  clustering.extract(cluster_indices);

  // ============================================================
  // 5. 计算每个 cluster 的中心和尺寸
  // ============================================================

  for (const auto & indices : cluster_indices)
  {
    if (indices.indices.empty())
    {
      continue;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    cluster->reserve(indices.indices.size());

    for (const auto index : indices.indices)
    {
      cluster->push_back(
        non_plane_cloud->points[index]);
    }

    cluster->width =
      static_cast<std::uint32_t>(cluster->points.size());

    cluster->height = 1;
    cluster->is_dense = true;

    // ---------- 包围盒 ----------
    pcl::PointXYZRGB min_point;
    pcl::PointXYZRGB max_point;

    pcl::getMinMax3D(
      *cluster,
      min_point,
      max_point);

    // ---------- 质心 ----------
    Eigen::Vector4f centroid;

    pcl::compute3DCentroid(
      *cluster,
      centroid);

    ClusterInfo info;

    info.point_count = cluster->points.size();

    info.center_x = centroid[0];
    info.center_y = centroid[1];
    info.center_z = centroid[2];

    info.size_x =
      static_cast<double>(max_point.x - min_point.x);

    info.size_y =
      static_cast<double>(max_point.y - min_point.y);

    info.size_z =
      static_cast<double>(max_point.z - min_point.z);

    // color

    info.color = classifier_.classifyColor(cluster);

    results.push_back(info);
  }

  return results;
}