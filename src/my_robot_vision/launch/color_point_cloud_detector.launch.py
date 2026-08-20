from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    detector_node = Node(
        package="my_robot_vision",
        executable="color_point_cloud_detector",
        output="screen",
        parameters=[
            {"point_cloud_topic": "/camera_link/points"},
            {"target_frame": "base_link"},
        ],
    )

    return LaunchDescription([
        detector_node,
    ])
