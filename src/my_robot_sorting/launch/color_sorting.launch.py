from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_robot_sorting",
            executable="color_sorting_node",
            output="screen",
            parameters=[
                {"wait_for_stable_detections": True},
                {"stable_detection_count": 3},
            ],
        ),
    ])
