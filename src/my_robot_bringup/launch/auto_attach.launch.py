import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'attach'
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'graspable_models.yaml'
    )

    # 定义要启动的节点
    auto_attach_node = Node(
        package=package_name,
        executable='auto_attach_node',
        name='auto_attach_node_instance', # 为这个节点实例起一个名字
        output='screen', # 将节点的输出打印到屏幕上
        parameters=[config] # 加载配置文件中的参数
    )

    return LaunchDescription([
        auto_attach_node,
    ])
