# 🤖 ROS2 MoveIt2 机械臂抓取分拣系统

> 基于 ROS 2 Humble + MoveIt2 + Gazebo 的工业机器人抓取分拣全流程仿真，涵盖 URDF 建模、运动规划、视觉检测、自动抓取与颜色分拣。

<!-- 
  TODO: 替换为你的实际演示 GIF/视频
  录制方法: 
  1. 启动 demo 后用 obs 录屏
  2. 转成 gif: ffmpeg -i demo.mp4 -vf "fps=15,scale=800:-1" demo.gif
  3. 放到 docs/ 目录或直接用 GitHub 链接
-->
<!-- ![demo](docs/demo.gif) -->

## ✨ 功能特性

| 模块 | 能力 | 技术栈 |
|------|------|--------|
| 🦾 运动规划 | 6-DOF 机械臂轨迹规划、碰撞检测、避障 | MoveIt2 / OMPL / KDL |
| 👁️ 视觉检测 | 点云分割 + 颜色识别，输出物体位姿 | PCL / TF2 |
| 🏭 自动分拣 | 根据颜色自动抓取 → 分类放置 | 行为树 / 状态机 |
| 🔗 物理附着 | Gazebo 仿真中模拟夹爪抓取 | AutoAttach Plugin |
| 🎮 仿真环境 | 完整 Gazebo 世界 + RViz 可视化 | Gazebo Classic / RViz2 |

## 📐 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      Gazebo 仿真环境                         │
│  ┌──────────┐    ┌──────────┐    ┌───────────────────────┐  │
│  │ 机械臂    │    │ 夹爪     │    │ 待分拣物体 (红/蓝/绿) │  │
│  │ (URDF)   │◄───│ (Joint)  │◄───│ (SDF Models)          │  │
│  └────┬─────┘    └──────────┘    └───────────────────────┘  │
│       │                                                     │
└───────┼─────────────────────────────────────────────────────┘
        │ /joint_states
        ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 节点层                              │
│                                                             │
│  ┌──────────────┐   ┌──────────────┐   ┌────────────────┐  │
│  │  Vision Node │   │ MoveIt2 Core │   │ Sorting Node   │  │
│  │  (PCL+Color) │──▶│ (Planning)   │◀──│ (State Machine)│  │
│  │              │   │              │   │                │  │
│  │ /detected_   │   │ /pose_command│   │ /open_gripper  │  │
│  │  objects     │   │ /joint_cmd   │   │ /close_gripper │  │
│  └──────────────┘   └──────────────┘   └────────────────┘  │
│                                                             │
│  ┌──────────────┐   ┌──────────────┐                       │
│  │  Commander   │   │  AutoAttach  │                       │
│  │  (C++ Node)  │   │  (Plugin)    │                       │
│  └──────────────┘   └──────────────┘                       │
└─────────────────────────────────────────────────────────────┘
```

**数据流：**
```
视觉检测 → /detected_objects → 分拣决策 → /pose_command → MoveIt2 规划 → 执行抓取 → AutoAttach 附着 → 放置到目标区域
```

## 📁 项目结构

```
src/
├── my_robot_description/       # 机器人 URDF/XACRO 模型
│   ├── urdf/                   #   关节定义、连杆、传感器
│   ├── meshes/                 #   3D 网格文件
│   └── config/                 #   控制器参数
├── my_robot_moveit_config/     # MoveIt2 运动规划配置
│   ├── config/                 #   SRDF、kinematics、planning
│   └── launch/                 #   MoveIt 启动文件
├── my_robot_bringup/           # 系统级启动文件
│   ├── demo.launch.py          #   一键启动（仿真+视觉+分拣）
│   ├── robot.launch.py         #   仿真环境启动
│   └── moveit_rviz.launch.py   #   RViz 可视化
├── my_robot_commander_cpp/     # 机械臂控制节点 (C++)
│   ├── src/                    #   位姿执行、夹爪控制
│   └── include/                #   头文件
├── my_robot_interfaces/        # 自定义消息/服务定义
│   ├── msg/                    #   DetectedObject, PoseCommand
│   └── srv/                    #   自定义服务接口
├── my_robot_vision/            # 视觉检测模块
│   ├── color_point_cloud_detector.py  # 点云+颜色检测
│   └── launch/                 #   检测启动文件
├── my_robot_sorting/           # 颜色分拣应用
│   ├── color_sorting_node.py   #   分拣状态机
│   └── launch/                 #   分拣启动文件
├── my_robot_test/              # 测试模型与节点
│   └── models/                 #   红/蓝方块、圆柱等
└── attach/                     # Gazebo AutoAttach 插件
```

## 🛠️ 环境要求

| 依赖 | 版本 |
|------|------|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill (LTS) |
| MoveIt2 | 2.x (Humble) |
| Gazebo | Classic 11 |
| PCL | ≥ 1.12 |
| 编译器 | GCC ≥ 11 |

## 🚀 快速开始

### 1. 安装依赖

```bash
# ROS 2 Humble（如未安装）
sudo apt install ros-humble-desktop

# MoveIt2
sudo apt install ros-humble-moveit

# Gazebo Classic
sudo apt install ros-humble-gazebo-ros-pkgs

# PCL
sudo apt install libpcl-dev
```

### 2. 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 一键启动

```bash
ros2 launch my_robot_bringup demo.launch.py
```

这会同时启动：Gazebo 仿真 + MoveIt2 + RViz + 机械臂控制 + AutoAttach + 视觉检测 + 测试物体。

### 4. 启动分拣

```bash
# 在另一个终端
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_sorting color_sorting.launch.py
```

机械臂将自动检测物体 → 规划路径 → 抓取 → 按颜色分类放置。

### 5. 单独调试

```bash
# 只启动仿真（不带 RViz）
ros2 launch my_robot_bringup robot.launch.py

# 单独启动 RViz
ros2 launch my_robot_bringup moveit_rviz.launch.py

# 加载测试物体
ros2 launch my_robot_test spawn_test_models.launch.py

# 启动视觉检测
ros2 launch my_robot_vision color_point_cloud_detector.launch.py

# 查看检测结果
ros2 topic echo /detected_objects --once

# RViz 中可视化标记
# 添加 Marker Display → 订阅 /detected_object_markers
```

## 📡 关键 Topics

| Topic | 类型 | 说明 |
|-------|------|------|
| `/detected_objects` | `DetectedObjectArray` | 检测到的物体列表（位姿+颜色） |
| `/detected_object_markers` | `visualization_msgs/MarkerArray` | RViz 可视化标记 |
| `/pose_command` | `PoseCommand` | 目标位姿指令 |
| `/open_gripper` | `std_msgs/Bool` | 夹爪打开指令 |
| `/close_gripper` | `std_msgs/Bool` | 夹爪关闭指令 |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态反馈 |

## 🧩 自定义消息

```bash
# DetectedObject — 单个检测物体
float64 x, y, z        # 物体中心坐标
string   color          # 颜色标签 (red/blue/green)
float64  confidence     # 检测置信度

# DetectedObjectArray — 物体数组
DetectedObject[] objects

# PoseCommand — 位姿命令
geometry_msgs/Pose pose
string             action   # "pick" / "place"
```

## 🧪 测试模型

`my_robot_test/models/` 目录下包含：

| 模型 | 说明 |
|------|------|
| `red_cube` | 红色立方体 |
| `blue_cube` | 蓝色立方体 |
| `grasp_cube` | 抓取测试立方体 |
| `blue_cylinder` | 蓝色圆柱体 |

## 📊 技术要点

### 运动规划
- 使用 MoveIt2 的 **OMPL** 规划器，支持 RRTConnect / PRM 等算法
- 配置了 **碰撞检测**（PlanningScene），避免机械臂自碰撞和环境碰撞
- **KDL** 运动学求解器，支持正/逆运动学

### 视觉检测
- 基于 **PCL** 的点云分割（VoxelGrid 降采样 + 欧式聚类）
- **颜色空间转换**（RGB → HSV）实现颜色分类
- TF2 坐标变换，将检测结果转换到机械臂基坐标系

### 抓取策略
- 预设抓取姿态 + 基于物体位姿的自适应调整
- 夹爪开合通过 Joint 模拟，AutoAttach 插件实现物理附着
- 分拣状态机管理：检测 → 抓取 → 运输 → 放置 → 回归

## 🔧 已知限制 & 后续计划

- [ ] 接入深度学习检测模型（YOLOv8 / PointNet）替代颜色识别
- [ ] 增加力/力矩传感器仿真，实现力控抓取
- [ ] 支持更多物体形状（圆柱、球体混合场景）
- [ ] 添加 BehaviorTree 编排复杂任务流
- [ ] 集成 GitHub Actions CI

## 📄 License

MIT

## 🙏 致谢

- [MoveIt2](https://moveit.ros.org/) — 运动规划框架
- [ROS 2](https://docs.ros.org/en/humble/) — 机器人操作系统
- [Gazebo](https://gazebosim.org/) — 物理仿真引擎
