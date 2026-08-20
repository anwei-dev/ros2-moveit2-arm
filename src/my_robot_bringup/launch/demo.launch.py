from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'robot.launch.py',
            ])
        ),
        launch_arguments={'start_rviz': 'true'}.items(),
    )

    spawn_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_test'),
                'launch',
                'spawn_test_models.launch.py',
            ])
        )
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_vision'),
                'launch',
                'color_point_cloud_detector.launch.py',
            ])
        )
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_models_launch],
    )

    delayed_vision = TimerAction(
        period=8.0,
        actions=[vision_launch],
    )

    return LaunchDescription([
        robot_launch,
        delayed_spawn,
        delayed_vision,
    ])
