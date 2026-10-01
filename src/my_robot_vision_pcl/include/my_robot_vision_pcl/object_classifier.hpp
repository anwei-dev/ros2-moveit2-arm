#ifndef MY_ROBOT_VISION_PCL__OBJECT_CLASSIFIER_HPP_
#define MY_ROBOT_VISION_PCL__OBJECT_CLASSIFIER_HPP_

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class ObjectClassifier
{
public:

  ObjectClassifier() = default;


  std::string classifyColor(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) const;


private:

  std::string classifyRGB(
    uint8_t r,
    uint8_t g,
    uint8_t b) const;

};


#endif