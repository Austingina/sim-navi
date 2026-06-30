# r1_pro × Isaac Sim × ROS 2

在 NVIDIA **Isaac Sim 5.1** 里把 **r1_pro**（swerve 底盘 + 机械臂）放进 3D 高斯泼溅(NuRec)
重建的真实场景，接入 **ROS 2 (Jazzy)**，打通传感器（RGBD 相机 / Mid360 雷达 / IMU /
真实地理对齐的 GPS）、底盘控制和 RViz 可视化。

> 详细的启动步骤、话题、GPS 原理、碰撞地图生成等，见 **[`ros2_sensors/README.md`](ros2_sensors/README.md)**。

## 目录结构

```
.
├── scene.usd                 # 主场景：引用机器人 + 碰撞 usdz，含 georef 元数据
├── r1_pro/                   # 机器人资产（URDF / mesh / usd），可直接 clone 即用
├── ros2_sensors/             # ROS 2 端：节点 / launch / RViz / 详细 README
│   ├── gps_publisher.py          # odom → /gps/fix（自动读 georef.json）
│   ├── base_controller.py        # /cmd_vel → /joint_command（swerve 运动学）
│   ├── setup_sensors.py          # Isaac Script Editor：建传感器 + ROS 发布图
│   ├── setup_control.py          # Isaac Script Editor：建 Articulation 控制图
│   ├── bringup.launch.py         # 起纯节点（GPS + 控制器）
│   ├── start_simulation.launch.py# 顶层一键：bringup + RViz
│   └── r1_pro.rviz
└── zhicheng-usdz/            # 场景碰撞地图 + 地理配准的离线生成工具
    ├── add_collision_to_usdz.py  # 给高斯 usdz 加隐藏碰撞网格 + 焊入 georef
    ├── make_georef.py            # 从重建 PLY 提取地理配准 → georef.json
    └── lcc-usdz-result/georef.json   # 地理配准单一可信源（已提交）
```

## 环境要求

- **Isaac Sim 5.1**（含 `isaacsim.ros2.bridge`、`omni.nurec`）
- **ROS 2 Jazzy**：`source /opt/ros/jazzy/setup.bash`，与 Isaac 同一 ROS 网络
- RViz 卫星底图：`sudo apt install ros-jazzy-rviz-satellite`
- 离线工具依赖：`pip install -r requirements.txt`（或用 Isaac 自带 python）

## 快速开始

1. Isaac Sim 打开 `scene.usd`，Script Editor 依次跑 `ros2_sensors/setup_sensors.py`、
   `setup_control.py`，然后 **Play**。
2. 终端（仓库根目录下）：
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 launch ros2_sensors/start_simulation.launch.py
   ```
3. 键盘遥控：`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

完整说明见 [`ros2_sensors/README.md`](ros2_sensors/README.md)。

## 资产与数据（**未纳入 Git**）

为避开 GitHub 100MB 限制，以下大文件被 `.gitignore` 排除，需另行获取/生成：

| 路径 | 内容 | 怎么来 |
|---|---|---|
| `zhicheng-usdz/lcc-usdz-result/*.usdz` | 高斯场景 + 带碰撞 usdz（~3.3GB） | 原始 usdz 来自重建；带碰撞版由 `add_collision_to_usdz.py` 生成 |
| `zhicheng/` | 重建原始数据（点云 ply + 网格 obj，~1.2GB） | 来自建图工具 L2Pro |
| `zhicheng-usdz/mesh-files/`, `zhicheng.usd`, `collision.usdc` | 中间产物 | 生成流程的中间文件 |

**已提交且关键**：`zhicheng-usdz/lcc-usdz-result/georef.json`（地理配准）、`r1_pro/`（机器人模型）、
`scene.usd`。拿到上述大资产后放回对应路径，打开 `scene.usd` 即可。
重新生成带碰撞地图与 georef 的流程见 [`ros2_sensors/README.md` 第 8 节](ros2_sensors/README.md)。

> ⚠️ 私有仓库：`zhicheng-usdz/` 工具源自公司内网 `slam-nav` 项目，重建数据为内部资产，请勿公开。
