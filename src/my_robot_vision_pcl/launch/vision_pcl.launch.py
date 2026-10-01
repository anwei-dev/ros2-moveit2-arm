from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_vision_pcl',
            executable='point_cloud_processor_node',
            name='point_cloud_processor',
            output='screen'
        ),

        Node(
            package='my_robot_vision_pcl',
            executable='object_detection_node',
            name='object_detection',
            output='screen'
        ),
    ])