#include "my_robot_commander_cpp/planning_scene_manager.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <cmath>
#include <functional>
#include <set>
#include <string>

PlanningSceneManager::PlanningSceneManager()
: Node("planning_scene_manager")
{
    // 接收视觉检测结果
    // 这里只保存检测结果，不立即修改 Planning Scene
    detected_objects_sub_ =
        this->create_subscription<
            my_robot_interfaces::msg::DetectedObjectPCLArray>(
            "/vision/detected_objects",
            10,
            std::bind(
                &PlanningSceneManager::detectedObjectsCallback,
                this,
                std::placeholders::_1));

    // 创建更新 Planning Scene 的 Service
    update_scene_service_ =
        this->create_service<std_srvs::srv::Trigger>(
            "/update_planning_scene",
            std::bind(
                &PlanningSceneManager::updatePlanningSceneCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    RCLCPP_INFO(
        this->get_logger(),
        "Planning Scene Manager started");

    RCLCPP_INFO(
        this->get_logger(),
        "Waiting for /update_planning_scene request");
}

void PlanningSceneManager::detectedObjectsCallback(
    const my_robot_interfaces::msg::DetectedObjectPCLArray::SharedPtr msg)
{
    latest_detected_objects_ = *msg;

    // 第一帧检测结果
    if (stable_frame_count_ == 0)
    {
        previous_detected_objects_ = *msg;
        stable_frame_count_ = 1;

        return;
    }

    // 与上一帧比较
    if (isSameDetection(
            previous_detected_objects_,
            *msg))
    {
        ++stable_frame_count_;
    }
    else
    {
        // 检测结果发生变化，重新开始计数
        stable_frame_count_ = 1;
    }

    previous_detected_objects_ = *msg;

    // 连续 3 帧稳定
    if (stable_frame_count_ >= 3)
    {
        stable_detected_objects_ = *msg;
        has_stable_detection_ = true;
    }
}

bool PlanningSceneManager::isSameDetection(
    const my_robot_interfaces::msg::DetectedObjectPCLArray& a,
    const my_robot_interfaces::msg::DetectedObjectPCLArray& b)
{
    // 物体数量不同
    if (a.objects.size() != b.objects.size())
    {
        return false;
    }

    // 检测结果为空
    if (a.objects.empty())
    {
        return true;
    }

    // 点云检测存在一定浮动
    // 允许位置和尺寸存在 5 mm 误差
    constexpr double tolerance = 0.005;

    for (std::size_t i = 0; i < a.objects.size(); ++i)
    {
        const auto& object_a = a.objects[i];
        const auto& object_b = b.objects[i];

        // ID 不同
        if (object_a.id != object_b.id)
        {
            return false;
        }

        // 颜色不同
        if (object_a.color != object_b.color)
        {
            return false;
        }

        // 中心位置变化超过 5 mm
        if (std::abs(
                object_a.center.x -
                object_b.center.x) > tolerance)
        {
            return false;
        }

        if (std::abs(
                object_a.center.y -
                object_b.center.y) > tolerance)
        {
            return false;
        }

        if (std::abs(
                object_a.center.z -
                object_b.center.z) > tolerance)
        {
            return false;
        }

        // 尺寸变化超过 5 mm
        if (std::abs(
                object_a.dimensions.x -
                object_b.dimensions.x) > tolerance)
        {
            return false;
        }

        if (std::abs(
                object_a.dimensions.y -
                object_b.dimensions.y) > tolerance)
        {
            return false;
        }

        if (std::abs(
                object_a.dimensions.z -
                object_b.dimensions.z) > tolerance)
        {
            return false;
        }
    }

    return true;
}

void PlanningSceneManager::updatePlanningSceneCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    RCLCPP_INFO(
        this->get_logger(),
        "Received request to update Planning Scene");

    if (!has_stable_detection_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "No stable detection available, "
            "skipping Planning Scene update");

        response->success = false;
        response->message =
            "No stable detection available";

        return;
    }

    // 使用连续 3 帧稳定后的结果
    updatePlanningScene();

    response->success = true;
    response->message =
        "Planning Scene updated successfully";
}

void PlanningSceneManager::updatePlanningScene()
{
    // 当前这一轮稳定检测到的所有物体 ID
    std::set<std::string> detected_object_ids;

    for (const auto& object :
         stable_detected_objects_.objects)
    {
        const std::string object_id =
            std::to_string(object.id);

        detected_object_ids.insert(object_id);

        // DetectedObjectPCL 中的 center
        // 表示物体几何中心
        const double center_x =
            object.center.x;

        const double center_y =
            object.center.y;

        const double center_z =
            object.center.z;

        // dimensions 直接作为 Box 尺寸
        const double size_x =
            object.dimensions.x;

        const double size_y =
            object.dimensions.y;

        const double size_z =
            object.dimensions.z;

        // Planning Scene 中还没有这个物体
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
            // 已经存在，更新位置和尺寸
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

    // 检查之前存在、现在已经消失的物体
    for (const auto& old_id :
         current_object_ids_)
    {
        if (detected_object_ids.find(old_id) ==
            detected_object_ids.end())
        {
            removeObject(old_id);
        }
    }

    // 保存这一次写入 Planning Scene 的物体 ID
    current_object_ids_ =
        detected_object_ids;

    RCLCPP_INFO(
        this->get_logger(),
        "Planning Scene update finished, %zu objects",
        detected_object_ids.size());
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

    collision_object.header.frame_id =
        "base_link";

    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type =
        shape_msgs::msg::SolidPrimitive::BOX;

    primitive.dimensions.resize(3);

    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_X] =
        size_x;

    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_Y] =
        size_y;

    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_Z] =
        size_z;

    geometry_msgs::msg::Pose box_pose;

    box_pose.orientation.w = 1.0;

    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(
        primitive);

    collision_object.primitive_poses.push_back(
        box_pose);

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
    planning_scene_interface_.removeCollisionObjects(
        {id});

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