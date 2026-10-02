#ifndef MY_ROBOT_COMMANDER_CPP__PLANNING_SCENE_MANAGER_HPP_
#define MY_ROBOT_COMMANDER_CPP__PLANNING_SCENE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/msg/detected_object_pcl_array.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <std_srvs/srv/trigger.hpp>

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

    void removeObject(
        const std::string& id);

    void updateBox(
        const std::string& id,
        double x,
        double y,
        double z,
        double size_x,
        double size_y,
        double size_z);

private:
    // 接收视觉检测结果
    // 连续稳定 3 帧后保存为稳定检测结果
    void detectedObjectsCallback(
        const my_robot_interfaces::msg::DetectedObjectPCLArray::SharedPtr msg);

    // 判断两帧检测结果是否相同
    bool isSameDetection(
        const my_robot_interfaces::msg::DetectedObjectPCLArray& a,
        const my_robot_interfaces::msg::DetectedObjectPCLArray& b);

    // /update_planning_scene Service 回调
    void updatePlanningSceneCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // 根据稳定的视觉结果更新 Planning Scene
    void updatePlanningScene();

    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface_;

    // 视觉检测结果订阅
    rclcpp::Subscription<
        my_robot_interfaces::msg::DetectedObjectPCLArray>::SharedPtr
        detected_objects_sub_;

    // 请求更新 Planning Scene 的 Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        update_scene_service_;

    // 最新一帧视觉检测结果
    my_robot_interfaces::msg::DetectedObjectPCLArray
        latest_detected_objects_;

    // 上一帧视觉检测结果
    my_robot_interfaces::msg::DetectedObjectPCLArray
        previous_detected_objects_;

    // 连续稳定的检测结果
    my_robot_interfaces::msg::DetectedObjectPCLArray
        stable_detected_objects_;

    // 当前连续稳定帧数
    int stable_frame_count_ = 0;

    // 是否已经获得稳定检测结果
    bool has_stable_detection_ = false;

    // 当前已经存在于 Planning Scene 中的物体 ID
    std::set<std::string> current_object_ids_;
};

#endif  // MY_ROBOT_COMMANDER_CPP__PLANNING_SCENE_MANAGER_HPP_