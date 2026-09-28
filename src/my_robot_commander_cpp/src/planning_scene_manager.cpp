#include "my_robot_commander_cpp/planning_scene_manager.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <functional>
#include <set>

PlanningSceneManager::PlanningSceneManager()
: Node("planning_scene_manager")
{
    detected_objects_sub_ =
        this->create_subscription<
            my_robot_interfaces::msg::DetectedObjectArray>(
            "/detected_objects",
            10,
            std::bind(
                &PlanningSceneManager::detectedObjectsCallback,
                this,
                std::placeholders::_1));
}

void PlanningSceneManager::detectedObjectsCallback(
    const my_robot_interfaces::msg::DetectedObjectArray::SharedPtr msg)
{
    // 当前这一帧检测到的 target 物体 ID
    std::set<std::string> detected_object_ids;

    for (const auto& object : msg->objects)
    {
        // 目前只把 target 类型的物体加入 Planning Scene
        if (object.shape != "target")
        {
            continue;
        }

        detected_object_ids.insert(object.id);

        // top_center 表示物体顶部中心，
        // 因此几何中心 z = top_center.z - height / 2
        const double center_x = object.top_center.x;
        const double center_y = object.top_center.y;
        const double center_z =
            object.top_center.z - object.height / 2.0;

        // 当前先使用 Box 对障碍物进行近似
        const double size_x = object.diameter_x;
        const double size_y = object.diameter_x;
        const double size_z = object.height;

        // 如果上一帧没有这个物体，说明是新出现的物体
        if (current_object_ids_.find(object.id) ==
            current_object_ids_.end())
        {
            addBox(
                object.id,
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
                object.id,
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