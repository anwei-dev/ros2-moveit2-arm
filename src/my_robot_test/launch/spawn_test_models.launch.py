# 用于加载测试模型
# 添加可被抓取的模型需要在 /attach/config/graspable_models.yanml 中添加模型名称
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_test = get_package_share_directory('my_robot_test')
    cube_sdf_file = os.path.join(pkg_test, 'models', 'grasp_cube', 'model.sdf')
    blue_cube_sdf_file = os.path.join(pkg_test, 'models', 'blue_cube', 'model.sdf')
    red_cube_sdf_file = os.path.join(pkg_test, 'models', 'red_cube', 'model.sdf')
    frustum_sdf_file = os.path.join(pkg_test, 'models', 'frustum_obstacle', 'model.sdf')
    blue_cylinder_sdf_file = os.path.join(pkg_test, 'models', 'blue_cylinder', 'model.sdf')

    spawn_cube = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'grasp_cube',
            '-file', cube_sdf_file,
            '-x', '0.8',
            '-y', '0.0',
            '-z', '0.06',
        ],
        output='screen'
    )

    spawn_frustum = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'frustum_obstacle',
            '-file', frustum_sdf_file,
            '-x', '0.45',
            '-y', '0.45',
            '-z', '0.0',
        ],
        output='screen'
    )

    # spawn_blue_cube = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'blue_cube',
    #         '-file', blue_cube_sdf_file,
    #         '-x', '-4.0',
    #         '-y', '-1.0',
    #         '-z', '4.0',
    #     ],
    #     output='screen'
    # )

    spawn_red_cube = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'red_cube',
            '-file', red_cube_sdf_file,
            '-x', '0.8',
            '-y', '0.3',
            '-z', '0.0',
        ],
        output='screen'
    )

    spawn_blue_cube_q4_a = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'blue_cube_q4_a',
            '-file', blue_cube_sdf_file,
            '-x', '0.92',
            '-y', '-0.18',
            '-z', '0.06',
        ],
        output='screen'
    )

    spawn_blue_cube_q4_b = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'blue_cube_q4_b',
            '-file', blue_cube_sdf_file,
            '-x', '0.72',
            '-y', '-0.26',
            '-z', '0.06',
        ],
        output='screen'
    )

    spawn_blue_cylinder = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'blue_cylinder',
            '-file', blue_cylinder_sdf_file,
            '-x', '0.8',
            '-y', '-0.5',
            '-z', '0.0',
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_cube,
        spawn_frustum,
        # spawn_blue_cube,
        spawn_red_cube,
        spawn_blue_cube_q4_a,
        spawn_blue_cube_q4_b,
        spawn_blue_cylinder,
    ])
