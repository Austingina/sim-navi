# r1_pro × Isaac Sim × ROS 2

在 NVIDIA **Isaac Sim 5.1** 里把 **r1_pro**（swerve 底盘 + 机械臂）放进 3D 高斯泼溅(NuRec)
重建的真实场景，接入 **ROS 2 (Humble)**，打通传感器（RGBD 相机 / Mid360 雷达 / IMU /
真实地理对齐的 GPS）、底盘控制和 RViz 可视化。

> 详细的启动步骤、话题、GPS 原理、碰撞地图生成等，见 **[`ros2_sensors/README.md`](ros2_sensors/README.md)**。

## 目录结构

```
.
├── scene.usd                 # 主场景：引用机器人 + 碰撞 usdz，含 georef 元数据
├── open_isaac_scene.sh       # 一键启动 Isaac Sim 并直接打开 scene.usd（支持 --headless 串流）
├── r1_pro/                   # 机器人资产（URDF / mesh / usd），可直接 clone 即用
├── ros2_sensors/             # ROS 2 端：节点 / launch / RViz / 详细 README
│   ├── gps_publisher.py          # odom → /gps/fix（自动读 georef.json）
│   ├── base_controller.py        # /cmd_vel → /joint_command（swerve 运动学）
│   ├── setup_sensors.py          # Isaac Script Editor：建传感器 + ROS 发布图
│   ├── setup_control.py          # Isaac Script Editor：建 Articulation 控制图
│   ├── bringup.launch.py         # 起纯节点（GPS + 控制器）
│   └── r1_pro.rviz
├── scene_tools/             # 场景碰撞地图 + 地理配准的离线生成工具
│   ├── add_collision_to_usdz.py  # 给高斯 usdz 加隐藏碰撞网格 + 焊入 georef
│   ├── make_georef.py            # 从重建 PLY 提取地理配准 → georef.json
│   └── georef.json               # 地理配准单一可信源（已提交）
└── assets/                   # 所有大资产，按场景分目录收纳（**未纳入 Git**）
    └── zhichengAB/                   # 一个场景一个文件夹（以后加新场景同理）
        ├── zhichengAB-collision.usdz     # 成品：带碰撞 + georef 的场景（scene.usd 引用）
        ├── lcc-usdz-result/
        │   └── zhichengAB.usdz               # 原始高斯场景（含 ~1.6GB .nurec）
        ├── mesh-files/
        │   └── zhichengAB.obj                # 网格（碰撞体来源）
        └── PLY/                              # 3DGS 训练输出（cfg_args / cameras.json / 分块 PLY）
            └── point_cloud/iteration_100/
                └── point_cloud.ply              # 点云（含 georef 头信息）
```

## 环境要求

- **Isaac Sim 5.1**（含 `isaacsim.ros2.bridge`、`omni.nurec`）
- **ROS 2 Humble**：`source /opt/ros/humble/setup.bash`，与 Isaac 同一 ROS 网络
- RViz 卫星底图：`sudo apt install ros-humble-rviz-satellite`
- 离线工具依赖：`pip install -r requirements.txt`（或用 Isaac 自带 python）

## 快速开始

1. 启动 Isaac Sim 并直接打开 `scene.usd`，然后点 **Play**（传感器图/控制图已固化在 USD 里，
   开箱即用；只有改传感器/控制配置时才需重跑 `ros2_sensors/setup_sensors.py` / `setup_control.py`）。
   - 手动：正常打开 Isaac Sim 后 `File > Open` 选 `scene.usd`；
   - 一键（推荐）：仓库根目录下 `./open_isaac_scene.sh`，启动即自动加载 `scene.usd`；
     加 `--headless` 则无头运行（WebRTC 串流，用 Isaac Sim Streaming Client 远程查看）。
     ```bash
     ./open_isaac_scene.sh                    # 有窗口
     ./open_isaac_scene.sh --headless         # 无头 + 串流
     ./open_isaac_scene.sh /path/to/其它.usd  # 打开别的 usd
     ```
     > 原理：`open_isaac_scene.sh` 调 `isaac-sim.sh --/app/file/openPath=<usd 绝对路径>`
     > 启动时自动打开场景（`--headless` 则换成 `isaac-sim.streaming.sh` 走 WebRTC），
     > 并顺带跳过一处 RTX 驱动版本误判。若以 root 运行需在脚本命令末尾加 `--allow-root`。
     > （`scene_tools/run_isaacsim*.sh` 是旧机器留下的同类脚本，路径指向不存在的 `~/isim`，已废弃。）
2. 终端（仓库根目录下）：
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch ros2_sensors/start_simulation.launch.py
   ```
3. 键盘遥控：`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

完整说明见 [`ros2_sensors/README.md`](ros2_sensors/README.md)。

## 资产与数据（**未纳入 Git**）

所有大资产按场景统一放在 `assets/<场景名>/`，被 `.gitignore` 整目录排除，需另行获取/生成

| 路径 | 内容 | 怎么来 |
|---|---|---|
| `assets/zhichengAB/zhichengAB-collision.usdz` | 成品：带碰撞 + georef 的场景（~1.7GB，`scene.usd` 引用） | 由 `add_collision_to_usdz.py` 生成 |
| `assets/zhichengAB/PLY/point_cloud/iteration_100/point_cloud.ply` | 点云（含 georef 头信息，~3.4GB，SH 3 阶） | 来自建图工具 L2Pro / 3DGS |
| `assets/zhichengAB/mesh-files/zhichengAB.obj` | 网格（碰撞体来源，~0.2GB） | 来自建图工具 L2Pro |
| `assets/zhichengAB/lcc-usdz-result/zhichengAB.usdz` | 原始高斯场景（~1.6GB） | 来自建图工具 L2Pro |

**已提交且关键**：`scene_tools/georef.json`（地理配准）、`r1_pro/`（机器人模型）、`scene.usd`。
**最简部署**：只需拿到 `assets/zhichengAB/zhichengAB-collision.usdz`，放回原路径，打开 `scene.usd` 即可
（其余原始三件套仅在需要重新生成碰撞地图时才用）。
重新生成带碰撞地图与 georef 的流程见 [`ros2_sensors/README.md` 第 8 节](ros2_sensors/README.md)。
