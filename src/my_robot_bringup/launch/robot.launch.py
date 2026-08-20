from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Fixed startup sequence: Gazebo -> MoveIt move_group -> commander (RViz optional).
def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'gazebo.launch.py',
            ])
        )
    )

    commander_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'commander.launch.py',
            ])
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        )
    )

    auto_attach_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'auto_attach.launch.py',
            ])
        )
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'moveit_rviz.launch.py',
            ])
        ),
        condition=IfCondition(start_rviz),
    )

    delayed_move_group = TimerAction(
        period=5.0,
        actions=[move_group_launch],
    )

    delayed_commander = TimerAction(
        period=5.0,
        actions=[commander_launch],
    )

    delayed_attach = TimerAction(
        period=5.0,
        actions=[auto_attach_launch],
    )

    delayed_rviz = TimerAction(
        period=3.0,
        actions=[moveit_rviz_launch],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start RViz together with robot launch',
        ),
        gazebo_launch,
        delayed_move_group,
        delayed_commander,
        delayed_attach,
        delayed_rviz,
    ])
