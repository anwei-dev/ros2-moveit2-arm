#include "my_robot_vision_pcl/object_classifier.hpp"

#include <algorithm>
#include <vector>


std::string ObjectClassifier::classifyColor(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) const
{
  if (!cloud || cloud->empty())
  {
    return "unknown";
  }


  std::vector<int> reds;
  std::vector<int> greens;
  std::vector<int> blues;


  reds.reserve(cloud->size());
  greens.reserve(cloud->size());
  blues.reserve(cloud->size());


  for (const auto & point : cloud->points)
  {
    reds.push_back(point.r);
    greens.push_back(point.g);
    blues.push_back(point.b);
  }


  auto getMedian =
    [](std::vector<int> & values)
    {
      std::sort(
        values.begin(),
        values.end());

      return values[values.size() / 2];
    };


  int r = getMedian(reds);
  int g = getMedian(greens);
  int b = getMedian(blues);


  return classifyRGB(
    static_cast<uint8_t>(r),
    static_cast<uint8_t>(g),
    static_cast<uint8_t>(b));
}



std::string ObjectClassifier::classifyRGB(
  uint8_t r,
  uint8_t g,
  uint8_t b) const
{

  if (r > g * 1.5 && r > b * 1.5)
  {
    return "red";
  }


  if (b > r * 1.5 && b > g * 1.5)
  {
    return "blue";
  }


  if (g > r * 1.5 && g > b * 1.5)
  {
    return "green";
  }


  return "unknown";
}