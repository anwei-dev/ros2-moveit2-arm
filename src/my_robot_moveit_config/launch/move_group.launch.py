import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 构建基础配置 (不使用 generate_move_group_launch 封装)
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .to_moveit_configs()
    )

    # 2. 手动创建 move_group 节点
    # 这样我们可以 100% 确认参数被传入
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}, # 强制开启仿真时间
            {"trajectory_execution.execution_timeout_monitor": False}, # 关闭超时监控
        ],
    )

    # 3. (可选) 如果你还需要启动其他关联节点，可以从 moveit_config 里获取
    # 但核心修复是上面的 move_group_node
    
    return LaunchDescription([
        move_group_node
    ])
