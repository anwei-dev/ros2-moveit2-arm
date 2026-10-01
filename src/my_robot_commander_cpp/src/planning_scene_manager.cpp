#include "my_robot_commander_cpp/planning_scene_manager.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <functional>
#include <set>
#include <string>

PlanningSceneManager::PlanningSceneManager()
: Node("planning_scene_manager")
{
    detected_objects_sub_ =
        this->create_subscription<
            my_robot_interfaces::msg::DetectedObjectPCLArray>(
            "/vision/detected_objects",
            10,
            std::bind(
                &PlanningSceneManager::detectedObjectsCallback,
                this,
                std::placeholders::_1));
}

void PlanningSceneManager::detectedObjectsCallback(
    const my_robot_interfaces::msg::DetectedObjectPCLArray::SharedPtr msg)
{
    // 当前这一帧检测到的所有物体 ID
    std::set<std::string> detected_object_ids;

    for (const auto& object : msg->objects)
    {
        const std::string object_id =
            std::to_string(object.id);

        detected_object_ids.insert(object_id);

        // DetectedObjectPCL 中的 center
        // 已经表示物体的几何中心
        const double center_x = object.center.x;
        const double center_y = object.center.y;
        const double center_z = object.center.z;

        // DetectedObjectPCL 中的 dimensions
        // 直接作为碰撞 Box 的尺寸
        const double size_x = object.dimensions.x;
        const double size_y = object.dimensions.y;
        const double size_z = object.dimensions.z;

        // 如果上一帧没有这个物体，说明是新出现的物体
        if (current_object_ids_.find(object_id) ==
            current_object_ids_.end())
        {
            addBox(
                object_id,
                center_x,
                center_y,
                center_z,
                size_x,
                size_y,
                size_z);
        }
        else
        {
            // 已经存在，更新其位置和尺寸
            updateBox(
                object_id,
                center_x,
                center_y,
                center_z,
                size_x,
                size_y,
                size_z);
        }
    }

    // 检查上一帧存在、这一帧已经消失的物体
    for (const auto& old_id : current_object_ids_)
    {
        if (detected_object_ids.find(old_id) ==
            detected_object_ids.end())
        {
            removeObject(old_id);
        }
    }

    // 保存当前帧的物体 ID，供下一帧比较
    current_object_ids_ = detected_object_ids;
}

void PlanningSceneManager::addBox(
    const std::string& id,
    double x,
    double y,
    double z,
    double size_x,
    double size_y,
    double size_z)
{
    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.header.frame_id = "base_link";
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);

    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_X] = size_x;

    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_Y] = size_y;

    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_Z] = size_z;

    geometry_msgs::msg::Pose box_pose;

    box_pose.orientation.w = 1.0;

    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    collision_object.operation =
        moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObject(
        collision_object);

    RCLCPP_INFO(
        this->get_logger(),
        "Added box obstacle '%s' at "
        "(%.3f, %.3f, %.3f), "
        "size=(%.3f, %.3f, %.3f)",
        id.c_str(),
        x,
        y,
        z,
        size_x,
        size_y,
        size_z);
}

void PlanningSceneManager::removeObject(
    const std::string& id)
{
    planning_scene_interface_.removeCollisionObjects({id});

    RCLCPP_INFO(
        this->get_logger(),
        "Removed collision object '%s'",
        id.c_str());
}

void PlanningSceneManager::updateBox(
    const std::string& id,
    double x,
    double y,
    double z,
    double size_x,
    double size_y,
    double size_z)
{
    addBox(
        id,
        x,
        y,
        z,
        size_x,
        size_y,
        size_z);

    RCLCPP_INFO(
        this->get_logger(),
        "Updated box obstacle '%s' at "
        "(%.3f, %.3f, %.3f)",
        id.c_str(),
        x,
        y,
        z);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PlanningSceneManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}