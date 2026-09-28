#ifndef MY_ROBOT_COMMANDER_CPP__PLANNING_SCENE_MANAGER_HPP_
#define MY_ROBOT_COMMANDER_CPP__PLANNING_SCENE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/msg/detected_object_array.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <set>
#include <string>

class PlanningSceneManager : public rclcpp::Node
{
public:
    PlanningSceneManager();

    void addBox(
        const std::string& id,
        double x,
        double y,
        double z,
        double size_x,
        double size_y,
        double size_z);

    void removeObject(const std::string& id);

    void updateBox(
        const std::string& id,
        double x,
        double y,
        double z,
        double size_x,
        double size_y,
        double size_z);

private:
    void detectedObjectsCallback(
        const my_robot_interfaces::msg::DetectedObjectArray::SharedPtr msg);

    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface_;

    rclcpp::Subscription<
        my_robot_interfaces::msg::DetectedObjectArray>::SharedPtr
        detected_objects_sub_;

    // 上一帧已经加入 Planning Scene 的物体 ID
    std::set<std::string> current_object_ids_;
};

#endif  // MY_ROBOT_COMMANDER_CPP__PLANNING_SCENE_MANAGER_HPP_