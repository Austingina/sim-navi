# r1_pro × Isaac Sim × ROS 2 接入

在 NVIDIA Isaac Sim 5.1 里把 **r1_pro** 机器人放进高斯泼溅(NuRec)重建场景，
接入 ROS 2 (Humble)，打通**传感器**（RGBD 相机 / Mid360 雷达 / IMU / GPS）、
**底盘控制**（swerve）和**真实地理对齐的 GPS**。

> ROS 端环境：`source /opt/ros/humble/setup.bash`，且与 Isaac Sim 同一 ROS 网络。
> Isaac 端脚本：在 `Window > Script Editor` 里粘贴运行（Play 前）。

---

## 1. 这套东西都做了什么

| 模块 | 说明 |
|---|---|
| URDF 修复 | 修正 mesh 路径、贴图路径；OBJ(Y-up) 加 `rpy=1.5708 0 0` 对齐 Z-up；涂装改黑 |
| 场景合成 | `../scene.usd`：引用 NuRec 环境(usdz) + payload 机器人，解决 usdz 自包含引用丢失 |
| 碰撞 | 导入后给碰撞网格加 `PhysicsCollisionAPI`（机器人）；场景碰撞用单独 collision usdz |
| 传感器 | `setup_sensors.py`：相机 + PhysX 雷达 + IMU + clock/odom/TF，全部 OmniGraph 发布 |
| 控制 | `setup_control.py`(Isaac侧 Articulation 控制图) + `base_controller.py`(swerve 运动学) |
| GPS | `gps_publisher.py`：odom 真值经 UTM 精确换算成经纬度，和真实地理/OSM 对齐 |
| 可视化 | `r1_pro.rviz`：点云 / 里程计 / TF / GPS 卫星底图 |
| 一键启动 | `bringup.launch.py`：GPS + 控制器（纯 ROS 节点），全带 `use_sim_time` |

---

## 2. 文件清单

| 文件 | 跑在哪 | 作用 |
|---|---|---|
| `setup_sensors.py` | Isaac Script Editor | 建相机/雷达/IMU + 各 ROS2 发布图（/clock /odom /tf /joint_states 等） |
| `setup_control.py` | Isaac Script Editor | 建 Articulation 控制图（订阅 `/joint_command`）+ 配关节 Drive |
| `base_controller.py` | 系统 ROS2 | swerve 运动学：`/cmd_vel` → `/joint_command` |
| `isaac_keyboard_teleop.py` | Isaac Script Editor | Isaac 内 WASD 键盘遥控（不经过 ROS teleop） |
| `gps_publisher.py` | 系统 ROS2 | `/odom` → `/gps/fix`（NavSatFix），自动读 georef.json 精确换算 |
| `lidar_self_filter.py` | 系统 ROS2 | 裁掉 mid360 扫到的机身自身点 → `/mid360/points_filtered` |
| `bringup.launch.py` | 系统 ROS2 | 起纯节点：GPS + 控制器 + 雷达自裁剪（不起 static_transform_publisher） |
| `r1_pro.rviz` | rviz2 | 可视化配置 |

---

## 3. 启动顺序

### 第 1 步：Isaac Sim 打开场景（Play 前）
打开 `../scene.usd`，直接点 **Play**（务必 Play，否则 `/clock` 不走、TF 会时间外推报错）。

> 懒人法：仓库根目录下 `./open_isaac_scene.sh`（加 `--headless` 走 WebRTC 串流），
> 启动 Isaac Sim 时自动加载 `scene.usd`，省去手动 `File > Open`。

> **传感器图和控制图已经固化保存在 `scene.usd` 里**（OmniGraph + 传感器 prim 都是 USD 持久化的），
> **平时开箱即用，不必再跑 `setup_sensors.py` / `setup_control.py`。**
>
> 这两个是**幂等的"建图脚本"**，只在**改配置**时才回去重跑对应那个，跑完记得存 `scene.usd`：
>
> | 改了什么 | 重跑 |
> |---|---|
> | 传感器（雷达线数 / 相机参数 / 话题名 / TF） | `setup_sensors.py` |
> | 控制（关节 Drive / 订阅话题 / 运动学） | `setup_control.py` |
> | 换机器人 prim 路径 / 重新导入机器人 / 图被误删 | 对应脚本（路径要和 Stage 里 `ROBOT_PRIM` 一致） |

### 第 2 步：一键起 ROS 周边 + 可视化（系统终端，仓库根目录下）
```bash
source /opt/ros/humble/setup.bash
ros2 launch ros2_sensors/start_simulation.launch.py
```

> 只要纯节点（无 GUI，跑导航/无头）用 `ros2 launch ros2_sensors/bringup.launch.py` 即可。

> **机器人/传感器 TF 由 Isaac 侧 `ActionGraph_robot` 统一发布**（`/tf` 上两个节点，职责不同、不重复）：
> - `PubRawTF`：`odom → base_link`（来自里程计，动态）
> - `PubTF`：`base_link →` 各子连杆（arms/torso/wheels/zed 等，动态；**必须设 parentPrim=base_link**，否则默认发 `world→base_link` 会和上面冲突）
> - `PubSensorTF`（`/tf_static`）：`base_link→mid360/imu`
> - `PubCamTF`（`/tf_static`）：`zed_link→zed_camera`
> GPS 用 `NavSatFix.frame_id="base_link"`；`gps_publisher.py` 另发 static `map→odom`（georef 旋转）。

### 第 3 步：可视化（可选，单独开 RViz 时）
```bash
rviz2 -d ros2_sensors/r1_pro.rviz --ros-args -p use_sim_time:=true
```

### 第 4 步：开动机器人（键盘遥控）

**方式 A — Isaac 内键盘（推荐试手，不用开 ROS teleop）**

1. 完成第 1 步 Play（场景里已有 `ActionGraph_control` 订阅 `/joint_command`）
2. 终端起周边节点时**关掉外部底盘控制器**（否则会抢 `/joint_command`）：
   ```bash
   ros2 launch ros2_sensors/bringup.launch.py with_controller:=false
   ```
3. Isaac：`Window > Script Editor` → 打开 `ros2_sensors/isaac_keyboard_teleop.py` → **Run**
4. **焦点放在 Isaac 视口**，按键：

| 键 | 作用 |
|---|---|
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 原地左转 / 右转 |
| `Q` / `E` | 左横移 / 右横移 |
| `Space` | 急停 |

停遥控：Script Editor 里执行 `stop_teleop()`。  
**不要**同时开下面的 ROS `teleop_twist_keyboard` + `base_controller`，会抢控制。

**方式 B — ROS 终端键盘（对接导航栈时用）**

先装键盘遥控包（只需一次）：
```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

新开一个终端，**焦点放在这个终端窗口上**（按键才生效）：
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

默认发 `/cmd_vel`，`base_controller.py` 会转成 swerve 的 `/joint_command`。

常用键（终端里会打印完整说明）：
| 键 | 作用 |
|---|---|
| `i` / `,` | 前进 / 后退 |
| `j` / `l` | 左转 / 右转 |
| `u` / `o` | 前进+左转 / 前进+右转 |
| `k` 或 `空格` | 停车 |
| `q` / `z` | 加 / 减 最大速度 |

> 也可用单条命令测试：`ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"`（发一次就停）。

---

## 4. 话题一览

| 话题 | 类型 | 来源 |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Isaac |
| `/joint_states` | sensor_msgs/JointState | Isaac |
| `/odom` | nav_msgs/Odometry | Isaac |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | Isaac(odom→base_link, base_link→mid360/imu, zed_link→zed_camera) + gps_publisher(map→odom) |
| `/zed/rgb/image_raw` | sensor_msgs/Image | Isaac 相机 |
| `/zed/depth/image_rect_raw` | sensor_msgs/Image | Isaac 相机 |
| `/zed/camera_info` | sensor_msgs/CameraInfo | Isaac 相机 |
| `/mid360/points` | sensor_msgs/PointCloud2 | Isaac PhysX 雷达（含机身自身点） |
| `/mid360/points_filtered` | sensor_msgs/PointCloud2 | `lidar_self_filter.py`（裁掉机身自身点，RViz 用这个） |
| `/imu` | sensor_msgs/Imu | Isaac IMU |
| `/gps/fix` | sensor_msgs/NavSatFix | `gps_publisher.py` |
| `/cmd_vel` | geometry_msgs/Twist | 你 / 导航栈 |
| `/joint_command` | sensor_msgs/JointState | `base_controller.py` |

---

## 5. GPS 地理对齐 — 全自动，无需标定

重建工具(L2Pro)在 `../assets/zhichengAB/PLY/point_cloud/iteration_100/point_cloud.ply` 头里写好了配准：
`offset`(局部原点的真实 UTM 坐标) + `epsg 32649`(UTM 49N) + `scale 1` + `shift 0`，
即 **局部坐标 = UTM − offset，尺度=1，轴向对齐 UTM**。

`scene_tools/make_georef.py` 把它落成单一可信源 `georef.json`，`gps_publisher.py`
启动时自动加载（同样的 georef 也焊进了 `scene.usd` 的 customLayerData）。换算：
```
UTM = origin + scale·R(yaw)·odom位移   (origin = offset + scale·R(yaw)·spawn)
lat, lon = UTM(49N) → WGS84            (闭式公式，无需 pyproj)
```
`yaw`/`scale` 也从 georef 文件读（缺省 yaw=0、scale=1，即 `zhichengAB` 场景）。

**你通常唯一要设的是机器人出生世界坐标 `spawn_x/spawn_y`**（默认 (5,0)，与 `scene.usd` 一致）：
```bash
ros2 launch .../bringup.launch.py spawn_x:=30 spawn_y:=-12
```

### 5.1 换场景（新区域）如何配准 —— 无需出门测 GPS
若新场景是**已配准旧场景里的一块区域**（如 `zhicheng-square`），不用去 OSM 量真实经纬度，
直接把新场景对齐到旧场景即可（旧场景那段 UTM 换算是现成的）：
1. 在**新、旧两个场景**里各读同样 **2~3 个真实地标**的世界 `(x,y)`（Play 后 `ros2 topic echo /odom --once`，
   世界坐标 = `spawn + odom`；或 Script Editor 读 `base_link` 的 world pose）。
2. 填进 `scene_tools/align_new_scene.py` 的 `OLD`/`NEW` 列表并跑：
   ```bash
   python3 scene_tools/align_new_scene.py
   ```
   它用 Umeyama 解「新→旧」的旋转/平移，打印 **fit RMSE**（越小越可信），
   并写出 `scene_tools/georef_square.json`（含 `yaw_deg`/`offset`/`scale`）。
3. 把这份 georef（含 `yaw_deg`）**焊进碰撞 usdz**，让配准随资产走（`gps_publisher` 仍读同名
   json，二者保持一致）：
   ```bash
   python3 scene_tools/bake_georef_into_usdz.py \
       --usdz assets/zhicheng-square/zhicheng-square-collision.usdz \
       --georef scene_tools/georef_square.json
   ```
4. 启动时指定该 georef：
   ```bash
   ros2 launch .../bringup.launch.py georef_json:=$PWD/scene_tools/georef_square.json
   ```
> `zhicheng-square`：新区域与旧场景**共享原点(平移=0)**，仅绕 Z **转 90°** 对齐 UTM/ENU
> （之前 GPS 偏方向就是缺这 90°）。scale=1。georef 已焊进 usdz 的 `customLayerData['georef']`，
> 与 `scene_tools/georef_square.json`、`scene_zhicheng_square.usd` 三处一致。
> （`align_new_scene.py` 三点实测 ≈91.17°、RMSE≈0.21m，取整用 90°、暂不加平移。）
换出生点后读新坐标（Play 前在 Script Editor 跑）：
```python
import omni.usd
from pxr import UsdGeom, Usd
stage = omni.usd.get_context().get_stage()
m = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        stage.GetPrimAtPath("/World/r1_pro_with_gripper"))
print("spawn_x, spawn_y =", *m.ExtractTranslation()[:2])
```

> `NavSatFix.frame_id` 用 **`base_link`**（GPS 传感器所在帧）：rviz_satellite 用它锚定瓦片，
> 填 `odom` 会让机器人比底图超前整段 odom 位移（已修）。
>
> ⚠️ 核对位置用 **WGS-84** 源（OSM / 谷歌卫星图）。高德/百度/腾讯 = GCJ-02（偏 50~500m
> 且随位置变化，会看起来像“越走越偏”），别拿来比。

---

## 6. 传感器说明

- **相机**：挂 `zed_link`，已修正朝向看 +X（正前方）。topic `/zed/*`。
- **Mid360 雷达**：用 **PhysX Generic Lidar**（`RangeSensorCreateLidar`），对**物理碰撞体**投射，
  不依赖 RTX 渲染几何 —— 所以**高斯泼溅场景里没碰撞代理的物体扫不到**，
  机器人/地面/有 CollisionAPI 的物体才有点。近似 Mid360：360°×59°、high_lod 多线。
  想扫到环境，需给场景碰撞代理 mesh 开 CollisionAPI（见 `../scene_tools/add_collision_to_usdz.py`）。
- **IMU**：`base_link` 上，topic `/imu`。
- **GPS**：Isaac 无原生 GPS，用 odom 真值换算（见第 5 节）。

---

## 7. 常见问题

| 现象 | 原因 / 解决 |
|---|---|
| 点云 `width: 0` | RTX/PhysX 之分 + 场景无碰撞几何；本项目已用 PhysX，确认目标物有 CollisionAPI |
| RViz 点云不显示 | TF `base_link→mid360` 由 setup_sensors 发 `/tf_static`(需重跑最新版)；或 QoS 设 **Best Effort** |
| `could not transform mid360 to odom` | 重跑最新 `setup_sensors.py`(它发 mid360/imu 的 /tf_static) |
| TF 树混乱 / `world` 和 `odom` 同时连 `base_link` | Isaac `PubTF` 未设 `parentPrim`（已修：须为 base_link）；Stop→Play 或重开 scene.usd |
| 雷达扫到机身自身 | PhysX 雷达打所有碰撞体、无法忽略指定 prim(FilteredPairsAPI/rangeOffset 都无效)；用 `lidar_self_filter.py` 裁掉机身足迹→看 `/mid360/points_filtered`，按 RViz 收紧 half_x/half_y/z_min/z_max |
| 机器人比 OSM 底图超前(走得越远越偏) | `NavSatFix.frame_id` 须是 `base_link` 不是 `odom`(已修)；填 odom 会超前整段 odom 位移 |
| `Lookup would require extrapolation into the future` | 时间源不一致 → 所有 ROS 节点 + RViz 加 `use_sim_time:=true`，且 Isaac 在 Play |
| GPS 位置整体平移 | `spawn_x/spawn_y` 没设成机器人真实出生点；offset/scale 由 georef.json 自动加载 |
| GPS 方向偏(走得越远越歪) | 换了新场景但没重新配准 yaw → 跑 `align_new_scene.py` 生成 `georef_square.json` 并用 `georef_json:=` 指定(见 5.1) |
| GPS 和高德对不上 | 高德是 GCJ-02，本项目是 WGS-84，用 OSM/谷歌卫星图核对 |
| `rviz_satellite` 报错/无底图 | 需 `sudo apt install ros-humble-rviz-satellite` + 联网下瓦片 |
| 机器人穿墙/掉地面 | 场景碰撞 usdz 没生成或没挂进 `scene.usd`（见第 8 节） |

---

## 8. 从 L2Pro 扫描结果生成「带 georef 的碰撞 usdz」

**为什么需要**：L2Pro 给的高斯泼溅(NuRec)`zhichengAB.usdz` 里只有一个负责"好看画面"的
Volume，**没有任何能参与物理的几何**——机器人会直接穿过去。要让机器人能撞墙、走楼道、
被雷达扫到，得把同坐标系下的三角网格作为**隐藏的碰撞体**挂进场景；同时把地理配准焊进去。

### 8.1 起点：L2Pro 扫描仪的 3 个原始产物
从扫描仪以及软件中得到：（都是同一坐标系、米制）：
| L2Pro 产物 | 是什么 | 用途 |
|---|---|---|
| `point_cloud.ply` | 3DGS 点云，**头部带地理配准**（offsetx/y/z、epsg 32649、scale 1） | 提取 georef → 经纬度 |
| `zhichengAB.obj` | 三角网格 | 做物理**碰撞体** |
| `zhichengAB.usdz` | 高斯泼溅场景（仅画面，含 ~1.6GB `.nurec`） | 渲染画面 + 重打包底座 |

### 8.2 放到约定路径
所有大资产按场景放在仓库根的 `assets/<场景名>/`，脚本默认按此布局找文件。本场景(zhichengAB)
的原始文件路径：
```
assets/zhichengAB/PLY/point_cloud/iteration_100/point_cloud.ply   # PLY（含 georef 头）
assets/zhichengAB/mesh-files/zhichengAB.obj                       # OBJ（碰撞体来源）
assets/zhichengAB/lcc-usdz-result/zhichengAB.usdz                 # 原始高斯 usdz
```

### 8.3 一条命令：生成带碰撞 + georef 的 usdz
```bash
cd scene_tools
python3 add_collision_to_usdz.py        # 需 usd-core + numpy（或用 Isaac 自带 python）
```
产出两样东西：
- `../assets/zhichengAB/zhichengAB-collision.usdz` —— **自包含**：高斯画面 + 隐藏碰撞网格 +
  焊入的 georef（`customLayerData`）。这就是 `scene.usd` 引用的那个文件。
- `scene_tools/georef.json` —— 地理配准单一可信源（随仓库跟踪），`gps_publisher.py`
  自动读（见第 5 节）。注意它落在工具目录、不在 gitignore 的 `assets/` 里。

> 地理配准是从 **PLY 头自动提取**的（脚本内部调 `make_georef.py`），**不需要手工标定**。
> 只想刷新 georef.json、不重打包 usdz：单独跑 `python3 make_georef.py`。

### 8.4 `add_collision_to_usdz.py` 做的 4 步（原理）
1. 解包原 usdz（本质是 zip），拿到根层 `default.usda` / `gauss.usda` / `.nurec`
2. 解析 OBJ，写成独立图层 `collision.usdc`：一个 `Mesh`，加 `PhysicsCollisionAPI` +
   `MeshCollisionAPI(approximation=none)`，默认 `visibility=invisible`（不挡画面、只参与物理）
3. 把 `collision.usdc` 挂成根层的 subLayer；同时从 PLY 提取地理配准，焊进根层
   `customLayerData['georef']` 并落 `georef.json`（见第 5 节，单一可信源）
4. `UsdUtils.CreateNewUsdzPackage` 跟随依赖（含 1.6GB `.nurec`）重新打包成自包含新 usdz

### 8.5 常用参数
| 参数 | 说明 |
|---|---|
| `--visible` | 让碰撞网格可见（灰色），首次目视检查它和高斯是否对齐；确认后去掉重生成 |
| `--approximation none` | 默认，原始三角面，静态建筑/可进出房间；面太多卡顿可换 `meshSimplification` |
| `--obj / --in / --out / --ply` | 自定义输入输出路径，默认即本项目布局 |

### 8.6 挂进仿真
`scene.usd` 已用 payload 引用碰撞 usdz（无需额外变换，同坐标系）：
```610:614:scene.usd
    def "zhichengAB_collision" (
        prepend payload = @./assets/zhichengAB/zhichengAB-collision.usdz@
    )
    {
    }
```
打开 `scene.usd` 即同时得到「高斯画面 + 隐藏碰撞 + 地理配准」。Mid360 雷达就是对这层
碰撞网格投射的（见第 6 节）。

> 单独刷新地理配准（不重打包 usdz）：`python3 make_georef.py` 重新从 PLY 生成 `georef.json`。
