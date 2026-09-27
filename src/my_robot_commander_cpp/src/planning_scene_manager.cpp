#include "my_robot_commander_cpp/planning_scene_manager.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

PlanningSceneManager::PlanningSceneManager()
: Node("planning_scene_manager")
{
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

    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size_x;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size_y;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = size_z;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObject(collision_object);

    RCLCPP_INFO(
        get_logger(),
        "Added box obstacle '%s' at (%.2f, %.2f, %.2f)",
        id.c_str(),
        x, y, z);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlanningSceneManager>();

    node->addBox(
        "test_obstacle",
        0.5, 0.0, 0.3,
        0.2, 0.2, 0.6);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
