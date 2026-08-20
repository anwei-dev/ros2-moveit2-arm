# My Robot - ROS 2 机器人抓取与分拣系统

基于 ROS 2 的机器人抓取与分拣系统，集成 MoveIt! 运动规划、Gazebo 仿真、视觉检测、自动抓取和分拣功能。

## 项目结构

```
src/
├── my_robot_description/     # 机器人 URDF/XACRO 模型
├── my_robot_moveit_config/   # MoveIt! 运动规划配置
├── my_robot_bringup/         # 启动文件与脚本
├── my_robot_commander_cpp/   # 机械臂控制节点
├── my_robot_interfaces/      # 自定义消息接口
├── my_robot_vision/          # 视觉检测
├── my_robot_sorting/         # 颜色分拣应用
├── my_robot_test/            # 测试模型与节点
└── attach/                   # Gazebo 自动抓取附着
```

## 依赖

- ROS 2 (Humble/Jazzy)
- MoveIt!
- Gazebo
- PCL (Point Cloud Library)
- TF2

## 编译

```bash
cd /home/anl/git/bot/ros2_ws
colcon build
source install/setup.bash
```

## 使用方法

### 一键启动

同时启动仿真环境（Gazebo + MoveIt + RViz + Commander + AutoAttach）、测试模型和视觉检测：

```bash
ros2 launch my_robot_bringup demo.launch.py
```

### 分步启动

**1. 启动仿真环境：**

```bash
ros2 launch my_robot_bringup robot.launch.py start_rviz:=true
```

不带 RViz：省略 `start_rviz:=true`，之后按需单独启动：

```bash
ros2 launch my_robot_bringup moveit_rviz.launch.py
```

**2. 加载测试模型：**

```bash
ros2 launch my_robot_test spawn_test_models.launch.py
```

**3. 启动视觉检测：**

```bash
ros2 launch my_robot_vision color_point_cloud_detector.launch.py
```

查看检测结果：

```bash
ros2 topic echo /detected_objects --once
```

在 RViz 中通过 Marker 订阅 `/detected_object_markers` 可查看物体中心点坐标。

**4. 启动分拣应用：**

分拣控制器读取视觉检测结果，自动规划并执行抓取与放置序列：

```bash
ros2 launch my_robot_sorting color_sorting.launch.py
```

> 数据流：`color_point_cloud_detector` → `/detected_objects` → `color_sorting_node` → `/pose_command`、`/open_gripper` → Commander 执行 → AutoAttach 完成物理附着

## 自定义消息

- `DetectedObject` — 检测到的物体
- `DetectedObjectArray` — 物体数组
- `PoseCommand` — 位姿命令

## 测试模型

`my_robot_test/models/` 目录下包含：
- `red_cube` — 红色立方体
- `blue_cube` — 蓝色立方体
- `grasp_cube` — 抓取测试立方体
- `blue_cylinder` — 蓝色圆柱体
