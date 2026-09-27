#ifndef MY_ROBOT_COMMANDER_CPP__PLANNING_SCENE_MANAGER_HPP_
#define MY_ROBOT_COMMANDER_CPP__PLANNING_SCENE_MANAGER_HPP_

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>

class PlanningSceneManager : public rclcpp::Node
{
public:
    PlanningSceneManager();

    void removeObject(const std::string& id);    

    void updateBox(
	const std::string& id,
	double x,
	double y,
	double z,
	double size_x,
	double size_y,
	double size_z);

    void addBox(
	const std::string& id,
	double x,
	double y,
	double z,
	double size_x,
	double size_y,
	double size_z);
private:
	moveit::planning_interface::PlanningSceneInterface
		planning_scene_interface_;
};

#endif
