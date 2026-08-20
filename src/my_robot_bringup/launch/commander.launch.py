import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 获取 MoveIt 的所有配置 (URDF, SRDF, Kinematics 等)
    # 这里的 package_name 确保和你之前的一致
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .to_moveit_configs()
    )

    # 2. 运行你的 C++ 节点
    commander_node = Node(
        package="my_robot_commander_cpp", # 你的包名
        executable="commander",           # 你的可执行文件名
        output="screen",
        parameters=[
            moveit_config.to_dict(),      # 将机器人的 URDF/SRDF 喂给这个节点
            {"use_sim_time": True},       # 确保启用仿真时间
        ],
    )

    return LaunchDescription([
        commander_node
    ])
