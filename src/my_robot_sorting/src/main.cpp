#include "my_robot_sorting/color_sorting_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<ColorSortingNode>());

  rclcpp::shutdown();

  return 0;
}
