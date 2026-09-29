#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "my_robot_vision_pcl/point_cloud_processor.hpp"

class PointCloudProcessorNode : public rclcpp::Node
{
public:
    PointCloudProcessorNode()
        : Node("point_cloud_processor")
    {
        input_topic_ = declare_parameter<std::string>(
            "input_topic",
            "/camera_link/points");

        output_topic_ = declare_parameter<std::string>(
            "output_topic",
            "/vision/filtered_points");

        processor_ = std::make_unique<PointCloudProcessor>(
            -1.0, 1.0, // X
            -1.0, 1.0, // Y
            0.1, 2.0,  // Z
            0.01);     // voxel size: 1 cm

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliable();
        qos.durability_volatile();

        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_,
            qos);

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            qos,
            std::bind(
                &PointCloudProcessorNode::pointCloudCallback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Point cloud processor started.");

        RCLCPP_INFO(
            get_logger(),
            "Input: %s",
            input_topic_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Output: %s",
            output_topic_.c_str());
    }

private:
    void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(*msg, *input);

        auto output = processor_->process(input);

        sensor_msgs::msg::PointCloud2 output_msg;

        pcl::toROSMsg(*output, output_msg);

        output_msg.header = msg->header;

        publisher_->publish(output_msg);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Input points: %zu, output points: %zu",
            input->points.size(),
            output->points.size());
    }

    std::string input_topic_;
    std::string output_topic_;

    std::unique_ptr<PointCloudProcessor> processor_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        subscription_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<PointCloudProcessorNode>());

    rclcpp::shutdown();

    return 0;
}