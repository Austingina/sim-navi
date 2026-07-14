# r1_pro × Isaac Sim × ROS 2

在 NVIDIA **Isaac Sim 5.1** 里把 **r1_pro**（swerve 底盘 + 机械臂）放进 3D 高斯泼溅(NuRec)
重建的真实场景，接入 **ROS 2 (Humble)**，打通传感器（RGBD 相机 / Mid360 雷达 / IMU /
真实地理对齐的 GPS）、底盘控制和 RViz 可视化。

> 详细的启动步骤、话题、GPS 原理、碰撞地图生成等，见 **[`ros2_sensors/README.md`](ros2_sensors/README.md)**。

## 目录结构

```
.
├── scene.usd                 # 主场景（zhichengAB）：引用机器人 + 碰撞 usdz，含 georef 元数据
├── scene_seg.usd             # NuRec 相机场景（无头默认针对它调好了频率/视口）
├── run_isaacsim.sh           # 【主入口】干净环境启动 Isaac：--gui / --stream / --headless
├── isaac_headless.py         # --headless 用的 standalone 无头运行器（自动 Play、只走 ROS）
├── open_isaac_scene.sh       # 旧脚本：仅在 GUI/串流里看画面，不接 ROS 管线
├── r1_pro/                   # 机器人资产（URDF / mesh / usd），可直接 clone 即用
├── ros2_sensors/             # ROS 2 端：节点 / launch / RViz / 详细 README
│   ├── gps_publisher.py          # odom → /gps/fix（自动读 georef.json）
│   ├── base_controller.py        # /cmd_vel → /joint_command（swerve 运动学）
│   ├── setup_sensors.py          # Isaac Script Editor：建传感器 + ROS 发布图（幂等建图脚本）
│   ├── setup_control.py          # Isaac Script Editor：建 Articulation 控制图
│   ├── lidar_self_filter.py      # /livox/lidar_raw → /livox/points（裁掉机身自身点）
│   ├── pc2_to_livox.py           # /livox/points → /livox/lidar（livox CustomMsg，给 FAST-LIO）
│   ├── bringup.launch.py         # 起纯节点（GPS + 控制器 + 雷达自裁剪 + CustomMsg 转换）
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
- **ROS 2 Humble**：`source /opt/ros/humble/setup.bash`
- **DDS 必须与 Isaac 一致**：系统默认 CycloneDDS（`~/.bashrc` 里 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`）。
  `run_isaacsim.sh` 已把 Isaac 也对齐到 CycloneDDS（Isaac 5.1 bridge 自带该库）并继承 `CYCLONEDDS_URI`。
  两边 RMW 或 `ROS_DOMAIN_ID` 不一致会**互不发现**（话题能 `ros2 topic list` 到，但收不到任何数据）。
- RViz 卫星底图：`sudo apt install ros-humble-rviz-satellite`
- 离线工具依赖：`pip install -r requirements.txt`（或用 Isaac 自带 python）

## 快速开始

用仓库根目录的 **`run_isaacsim.sh`** 启动 Isaac（它会退 conda、清库路径、把 Isaac 的 DDS
对齐到系统 CycloneDDS）。三种模式：

```bash
./run_isaacsim.sh --headless scene_seg.usd    # 【推荐】纯无头：自动 Play、只走 ROS、最省 GPU
./run_isaacsim.sh --gui      scene.usd         # 本机物理显示器开 GUI
./run_isaacsim.sh --stream   scene.usd         # 无头 + WebRTC 串流（Isaac Sim Streaming Client 远程看）
```

传感器图/控制图已固化在 USD 里，开箱即用；只有改传感器/控制配置时才回去重跑
`ros2_sensors/setup_sensors.py` / `setup_control.py`（幂等建图脚本，跑完存盘）。

无头模式常用环境变量（`scene_seg.usd` 已给好默认值，可用 env 覆盖）：

| 变量 | 默认 | 作用 |
|---|---|---|
| `ISAAC_PHYSICS_HZ` | 200 | 物理步频；IMU 与 `/clock` 都按物理步走 |
| `ISAAC_HZ` | 0 | 实时节流目标(Hz)；`0`=不节流跑满，设 `200` 则 RTF≈1 |
| `ISAAC_RENDER_HZ` | 10 | 渲染抽帧频率；雷达 / odom / tf / 相机随它 |
| `ISAAC_CLOCK_HZ` | 20 | `/clock` 发布频率（物理步 + Gate 均匀降频） |
| `ISAAC_VIEWPORT` | 0 | `0`=不渲染(只出 IMU/雷达等)；`1`=开相机渲染 |

再起 ROS 周边（**系统终端，需与 Isaac 同一 CycloneDDS 和 `ROS_DOMAIN_ID`**）：

```bash
source /opt/ros/humble/setup.bash
ros2 launch ros2_sensors/bringup.launch.py             # GPS + 控制器 + 雷达自裁剪 + CustomMsg 转换
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 键盘遥控
```

> 只想在 GUI 里看看画面、不接 ROS 管线时，可用旧脚本 `./open_isaac_scene.sh`（只调
> `isaac-sim.sh` 打开场景，不处理 DDS / 传感器发布）。

完整说明（话题、频率、DDS、TF、GPS、碰撞地图生成）见 [`ros2_sensors/README.md`](ros2_sensors/README.md)。

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
