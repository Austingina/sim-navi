# 大学城场景：宇树 Go2（Unitree 官方 Isaac 扩展）接入指南

**环境：** ROS 2 Humble + Isaac Sim 6.0.1 + Isaac Lab 3.0.0-beta2 + slam-nav + fast_lio2  
**地图：** 复用大学城 `daxuecheng` 碰撞 + `georef_daxuecheng.json`  
**机器人：** 宇树 Go2（官方 `unitree_model` USD + `unitree_rl_lab` 速度策略）

> **重要：** Unitree「官方 Isaac 扩展」指的是 **Isaac Lab 上的 `unitree_rl_lab`**（RL 运动控制），  
> **不是** 本仓库 r1_pro 那种 OmniGraph + `base_controller`（swerve）一键替换。  
> 接到大学城导航栈时，必须额外做 **ROS 2 话题桥**：消费 `/cmd_vel`，发布与现栈兼容的雷达/IMU。

---

## 〇、架构（一眼看懂）

```
大学城碰撞 USD (不变)
        +
官方 Go2 USD (unitree_model/Go2/usd/go2.usd)
        +
unitree_rl_lab「Unitree-Go2-Velocity」策略  ← 真正走路
        +
ROS2 桥 (本仓库 go2_* )                    ← 对接 Nav2 / FAST-LIO
        │
        ├─ 订阅 /cmd_vel  (或 /cmd_vel 经 CM 后的输出)
        ├─ 发布 /livox/lidar + /livox/imu   (或桥接重映射)
        └─ 发布 /odom_gt /clock（与现 bringup 对齐）
                │
                ▼
        start_outdoor.bash -s + campus_nav（逻辑不变）
```

| 层 | 用什么 | 本机状态 |
|---|---|---|
| 地图 | `assets/daxuecheng/*` + `georef_daxuecheng.json` | 已有 |
| Go2 模型 | HuggingFace `unitreerobotics/unitree_model` → `Go2/usd/go2.usd` | 见 `third_party/unitree_model/` |
| 官方扩展 | [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) | `~/unitree_rl_lab`（已做 Lab3 补丁） |
| Isaac Lab | `~/IsaacLab` v3.0.0-beta2 + conda `env_isaaclab` | 已装；`rl[rsl-rl]` 已装 |
| 策略冒烟 | `Unitree-Go2-Velocity` train + play（`--viz kit`） | **已验收：能开视口并看到 Go2** |
| 导航栈 | `start_outdoor.bash -s` | 已有；足迹/外参需 Go2 版参数（未做） |
| r1 swerve | `base_controller.py` | **禁用**，不可用于 Go2 |

官方文档标称：**Isaac Sim 5.1 + Isaac Lab 2.3**。本机为 **Isaac Sim 6.0.1 + Isaac Lab 3.0**，必须套用下文「Lab 3.0 兼容补丁」。

---

## 一、准备清单

- [x] `third_party/unitree_model/Go2/usd/go2.usd` 存在（`./scripts/fetch_unitree_go2_model.sh`）
- [x] `~/unitree_rl_lab` 中 `UNITREE_MODEL_DIR` 指向上述 `unitree_model` 根目录
- [x] Isaac Lab + `env_isaaclab`；`./isaaclab.sh -i rl[rsl-rl]`；能列出 `Unitree-Go2-Velocity`
- [x] Lab3 补丁已打进 `~/unitree_rl_lab`（见步骤 2）
- [x] 短训权重 + `./unitree_rl_lab.sh -p --task Unitree-Go2-Velocity` 能开 **Isaac Lab** 窗口并看到 Go2
- [ ] 已有大学城资产：`assets/daxuecheng/daxuecheng-collision.usdz`（场景层另验）
- [ ] DDS：`source setup_ros_local.sh`（对接 outdoor 时再用）

---

## 二、操作流程

### 步骤 1 — 下载官方 Go2 USD

```bash
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
# 推荐镜像 + curl 分文件（勿用旧版 hf download + HF_ENDPOINT，易半截失败）
HF_ENDPOINT=https://hf-mirror.com ./scripts/fetch_unitree_go2_model.sh
ls -lh third_party/unitree_model/Go2/usd/go2.usd
ls -lh third_party/unitree_model/Go2/usd/configuration/
```

来源：[unitreerobotics/unitree_model](https://huggingface.co/datasets/unitreerobotics/unitree_model)。

> **注意：** 必须有入口 `Go2/usd/go2.usd` 以及 physics/sensor；脚本支持断点续传。

### 步骤 2 — Isaac Lab + unitree_rl_lab（Lab 3.0）

#### 2.1 环境与依赖

```bash
conda activate env_isaaclab
export ISAACSIM_PATH=~/isaacsim/current
export EXP_PATH=$ISAACSIM_PATH/apps
# 勿在 conda activate 之前 source setup_python_env.sh（易 SRE module mismatch）

cd ~/IsaacLab
# Lab 3：RSL-RL 是可选 extra。错误写法 ./isaaclab.sh -i rsl_rl 会被当成未知 token 跳过
./isaaclab.sh -i rl[rsl-rl]
python -c "import rsl_rl; print(rsl_rl.__file__)"
# 期望：.../site-packages/rsl_rl/__init__.py

cd ~/unitree_rl_lab
./unitree_rl_lab.sh -i
./unitree_rl_lab.sh -l | grep -i go2
# 应看到 Unitree-Go2-Velocity
```

`UNITREE_MODEL_DIR` **已指向**本仓库 `third_party/unitree_model`（`~/unitree_rl_lab/.../assets/robots/unitree.py`）。若目录移动再改。

#### 2.2 Lab 3.0 兼容补丁（本机 `~/unitree_rl_lab` 已改）

| # | 位置 | 修改 |
|---|---|---|
| 1 | `scripts/rsl_rl/play.py` | `isaaclab.utils.pretrained_checkpoint` → `isaaclab_rl.utils.pretrained_checkpoint` |
| 2 | 各 `*_env_cfg.py` | `AdditiveUniformNoiseCfg` → `UniformNoiseCfg`（仍 `as Unoise`） |
| 3 | 全库 | `from isaaclab.utils import configclass` → `from isaaclab.utils.configclass import configclass`（否则 `import isaaclab_tasks` 后变成不可调用的 module） |
| 4 | 各 env cfg `__post_init__` | `self.sim.physx.gpu_max_rigid_patch_count=...` → `self.sim.physics = PhysxCfg(gpu_max_rigid_patch_count=...)`（并 `from isaaclab_physx.physics import PhysxCfg`） |
| 5 | `tasks/*/agents/rsl_rl_ppo_cfg.py` | 旧 `policy = RslRlPpoActorCriticCfg(...)` → `actor`/`critic = RslRlMLPModelCfg(...)`（rsl-rl 5.x） |
| 6 | `scripts/rsl_rl/train.py`、`play.py` | 建 `OnPolicyRunner` 前调用 `handle_deprecated_rsl_rl_cfg`；play 导出用 `runner.export_policy_to_jit/onnx`（勿再读 `alg.policy`） |
| 7 | `unitree_rl_lab.sh -p` | **自动加 `--viz kit`**。Lab 3 省略 `--viz` 默认 headless，**不弹窗口** |

上游若更新 `unitree_rl_lab`，请重新核对上表。

#### 2.3 训练 + 播放（默认地面冒烟，非大学城）

Nucleus **没有** `Unitree-Go2-Velocity` 预训练包（404）。不要把 `Isaac-Velocity-*-Unitree-Go2-v0` 的权重直接塞给本任务（观测定义不同）。

```bash
conda activate env_isaaclab
cd ~/unitree_rl_lab

# 短训冒烟（能出 model_*.pt 即可；稳走需更多 iteration，如 3000+）
./unitree_rl_lab.sh -t --task Unitree-Go2-Velocity --max_iterations 100 --num_envs 64
ls logs/rsl_rl/unitree_go2_velocity/*/model_*.pt

# 播放（脚本已带 --viz kit）
./unitree_rl_lab.sh -p --task Unitree-Go2-Velocity
# 等价：python scripts/rsl_rl/play.py --task Unitree-Go2-Velocity --viz kit
```

**视口注意：**

- 窗口标题一般为 **Isaac Lab 3.0.0**；可用 Alt+Tab 查找。
- **首次**开 Kit/RTX 会编译着色器（日志里 `Waiting for RtPso async compilation`），远程桌面（NoMachine 等）上可能黑屏/无响应几分钟，**勿急着 Ctrl+C**。
- 约 100 iter 的权重往往站不稳/趴地，属正常；加长训练后再 play。
- 无界面验收可录视频：  
  `python scripts/rsl_rl/play.py --task Unitree-Go2-Velocity --video --video_length 200 --num_envs 16`  
  输出在 `logs/rsl_rl/unitree_go2_velocity/<run>/videos/play/`。

**本机已验收：** `./unitree_rl_lab.sh -p --task Unitree-Go2-Velocity` 能打开窗口并看到宇树 Go2。

### 步骤 3 — 大学城 + Go2 场景层 + 传感器图

仓库提供：

| 文件 | 作用 |
|---|---|
| `scene_daxuecheng_go2.usd` | 大学城碰撞 + 官方 `go2.usd`；停用 r1；**根路径**停用 r1 的 `ActionGraph_*` |
| `ros2_sensors/setup_sensors_go2.py` | 给 `/World/go2/base` 挂 Mid360/IMU + ROS 图（无 ZED、无 swerve） |
| `ros2_sensors/bringup_go2.launch.py` | GPS + `pc2_to_livox`；**默认** `with_controller:=false` |

```bash
# 终端 A：不要 conda activate env_isaaclab（会污染系统 ros2 → SRE module mismatch）
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
source setup_ros_local.sh
./run_isaacsim.sh --gui scene_daxuecheng_go2.usd
# Window → Script Editor 运行 ros2_sensors/setup_sensors_go2.py，再点 Play
```

> `run_isaacsim.sh` 只在 **slam-nav 仓库根**；不在 `~/unitree_rl_lab`。

贴地：路面约 `z≈-0.42`；层内默认 `go2` 平移 `(0,0,0.03)`。若穿地/悬空，只改 `scene_daxuecheng_go2.usd` 里 `xformOp:translate` 的 z。

**验收：**

- Stage 有 `/World/go2/base`；根上 `/ActionGraph_control` 等为 **inactive**
- Play 后：`ros2 topic list` 见 `/clock /odom_gt /livox/lidar_raw /livox/imu /tf /tf_static`

> 仅加载 USD **不会走路**。走路靠步骤 4 的速度策略。

### 步骤 4 — `/cmd_vel` → Unitree-Go2-Velocity

`base_velocity` 指令与 `/cmd_vel` 同语义（机体 `vx, vy, wz`）。

| 方案 | 说明 | 本仓库状态 |
|---|---|---|
| **A. Lab play + `/cmd_vel`** | 默认平地 + 策略；ROS 写 `vel_command_b` | **已提供骨架** `scripts/lab_play_go2_cmd_vel.py` |
| **A2. Lab + 大学城地形** | 任务里替换 ground 为大学城 mesh | 后续 |
| **B. Isaac 场景 + 外挂策略** | `scene_daxuecheng_go2` + 关节驱动 | 工作量大，暂缓 |

冒烟（Lab 平地，确认指令接通）：

```bash
conda activate env_isaaclab
source /opt/ros/humble/setup.bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
cd ~/unitree_rl_lab
python ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/scripts/lab_play_go2_cmd_vel.py \
  --task Unitree-Go2-Velocity --viz kit --num_envs 1

# 另一终端
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

脚本日志应周期性打印 `[cmd_vel] vx=0.30 ...`。短训权重可能仍站不稳，属正常。

### 步骤 5 — ROS 2 bringup（对接 outdoor 栈）

| 话题 | 方向 | 来源 |
|---|---|---|
| `/cmd_vel` | Go2/Lab 订阅 | Nav2 CM 输出；**勿**开 swerve `base_controller` |
| `/livox/lidar_raw` | Isaac 发布 | `setup_sensors_go2` |
| `/livox/lidar` + `/livox/points` | bringup | `pc2_to_livox.py` |
| `/livox/imu` | Isaac 发布 | 与雷达同 `livox_frame` |
| `/clock` `/odom_gt` `/tf*` | Isaac 发布 | 同上 |
| `/gps/fix` | bringup | `gps_publisher` + `georef_daxuecheng.json` |

```bash
# 终端 B：新开 bash；同样不要 env_isaaclab
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
source setup_ros_local.sh
source ~/Desktop/fast_lio2/ros2_ws/install/setup.bash   # livox_ros_driver2
ros2 launch ros2_sensors/bringup_go2.launch.py
# 可选：ros2 launch ros2_sensors/bringup_go2.launch.py with_go2_bridge:=true
```

若已误开 `env_isaaclab`，先 `conda deactivate`，或新开终端；确认 `which python3` 不是 `.../env_isaaclab/bin/python3`，且 `ros2 -h` 能正常打印。

- `go2_topic_bridge.py`：仅当上游在 `/unitree_go2/*` 等非标准名时需要；官方 Isaac 传感器图路径可不启。
- 无头：`./run_isaacsim.sh --headless scene_daxuecheng_go2.usd`（已纳入大学城无头分支）。
### 步骤 6 — Nav2 / FAST-LIO 参数

复制并改：

```bash
# 建议新建（勿直接改坏 r1 参数）
cp ~/Desktop/fast_lio2/ros2_ws/src/FAST_LIO/config/param_outdoor_sim.yaml \
   ~/Desktop/fast_lio2/ros2_ws/src/FAST_LIO/config/param_outdoor_sim_go2.yaml
```

至少修改：

- footprint / `robot_radius`（Go2 约 0.35–0.4 m 量级，按实机改）
- 最大 `vx/wz`（对齐策略限幅）
- 雷达高度带（狗背雷达高度 ≠ r1）
- FAST-LIO 外参：新建 `mid360_sim_go2.yaml`

启动 outdoor 时传入：

```bash
# 示例：改 start_outdoor.bash 或直接 launch
nav_params:=.../param_outdoor_sim_go2.yaml
```

### 步骤 7 — 端到端（大学城导航）

与《大学城路线仿真操作指南》相同，替换机器人侧：

1. 起 Go2 + 大学城（Lab play 或 `scene_daxuecheng_go2.usd` + 策略）
2. `source setup_ros_local.sh` + bringup（无 swerve；georef 仍用 `georef_daxuecheng.json`）
3. `./start_outdoor.bash -s`（Go2 nav 参数）→ 等 heading LOCKED  
   - auto_drive 发 `/cmd_vel_smoothed` → CM → `/cmd_vel`；**Go2 必须订阅最终 `/cmd_vel`**
4. RViz 只用 **2D Goal Pose**，目标 >20 m
5. 检查 `ros2 topic hz /plan` 与 `/cmd_vel_nav`

---

## 三、本仓库文件一览

| 路径 | 说明 |
|---|---|
| `scripts/fetch_unitree_go2_model.sh` | 拉取官方 Go2 USD |
| `scripts/lab_play_go2_cmd_vel.py` | Lab 播放 + 订阅 `/cmd_vel` 写策略指令 |
| `third_party/unitree_model/Go2/` | 官方模型（大文件，勿强行 git 提交） |
| `scene_daxuecheng_go2.usd` | 大学城 + Go2；停用 r1 与根路径 ActionGraph_* |
| `ros2_sensors/setup_sensors_go2.py` | Go2 Mid360/IMU/ROS 传感器图 |
| `ros2_sensors/bringup_go2.launch.py` | 无 swerve 的 outdoor bringup |
| `ros2_sensors/go2_topic_bridge.py` | 非标准命名空间话题重映射 |
| `docs/大学城_Go2_Unitree官方扩展接入.md` | 本文 |

**本机另改（不在本仓库）：** `~/unitree_rl_lab` 的 Lab3 补丁与 `unitree_rl_lab.sh -p --viz kit`。

---

## 四、验收清单

- [x] `go2.usd` 已下载
- [x] `unitree_rl_lab` 能列出 / train / play `Unitree-Go2-Velocity`（Kit 视口可见 Go2）
- [x] `scene_daxuecheng_go2.usd`：Go2 足底约 z=-0.42；根 ActionGraph_* inactive
- [ ] GUI 跑通 `setup_sensors_go2` + `bringup_go2`（`/livox/lidar`、`/livox/imu` 有 hz）
- [ ] `lab_play_go2_cmd_vel.py` 下 `/cmd_vel` 非零时策略指令日志变化（狗移动视权重）
- [ ] outdoor Nav2 `active`；2D Goal 后 `/plan` + `/cmd_vel_nav` 有频率

---

## 五、常见问题

### Q1：只有 USD，狗站着不动？

正常。官方走路靠 **Unitree-Go2-Velocity 策略**，不是 `base_controller`。

### Q2：本机没有 `env_isaaclab`？

按 [Isaac Lab 安装指南](https://isaac-sim.github.io/IsaacLab/) 安装，再 `./unitree_rl_lab.sh -i`。  
仅有 Isaac Sim 6.0.1 **不够**跑官方 RL 扩展。

### Q3：Isaac 6.0.1 + Lab 3 与 unitree_rl_lab（标称 5.1）？

本机已用 **Lab 3 补丁**跑通 train/play 视口。上游未合入前，升级 `unitree_rl_lab` 后需重打补丁。

### Q4：play 不弹窗口 / 像卡死？

1. Lab 3 **必须** `--viz kit`（`unitree_rl_lab.sh -p` 已自动加）。省略则 headless。  
2. 首次 GUI：RTX 着色器编译（`Waiting for RtPso...`），远程桌面可能黑屏数分钟。  
3. `import rsl_rl` 失败：执行 `./isaaclab.sh -i rl[rsl-rl]`（不是 `-i rsl_rl`）。  
4. `KeyError: class_name`：agent cfg 仍是旧 `policy=`，需改成 `actor`/`critic`（补丁 #5）。  
5. Carb 设置勿写成裸 `--/rtx/...` 命令行（会被 argparse 当成未知参数）；若需要请用 `--kit_args=...`。

### Q5：能否继续用 `start_outdoor` 的 auto_drive？

可以，只要 Go2 侧订阅 **`/cmd_vel`**（CM 输出）。不要 deactivate `collision_monitor`。

### Q6：和社区 `isaac-go2-ros2` 的关系？

社区仓已带 `/unitree_go2/cmd_vel` 等 ROS 话题，集成更快，但 **不是** Unitree 官方扩展。  
本指南按官方 `unitree_rl_lab` + `unitree_model`。

### Q7：`./run_isaacsim.sh: No such file` 或 `SRE module mismatch`？

1. **No such file**：当前目录是 `~/unitree_rl_lab`。请 `cd` 到 slam-nav 根再跑。  
2. **SRE module mismatch**：在 `env_isaaclab`（或残留 Isaac `PYTHONPATH`）里跑了系统 `/opt/ros/humble/bin/ros2`。  
   - Isaac GUI / bringup / outdoor → **系统 bash + `setup_ros_local.sh`**  
   - Lab play / `lab_play_go2_cmd_vel.py` → 另开终端再 `conda activate env_isaaclab`

### Q9：bringup 报 `No module named 'numpy'` / `rclpy._rclpy_pybind11`？

终端在 **conda base / env_isaaclab**，`python3` 不是系统 3.10。Humble 只认 `/usr/bin/python3`。

```bash
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
source setup_ros_local.sh   # 会 deactivate conda，并把 /usr/bin 置顶
# 确认：which python3 → /usr/bin/python3
source ~/Desktop/fast_lio2/ros2_ws/install/setup.bash
ros2 launch ros2_sensors/bringup_go2.launch.py
```

验收终端也必须先 `cd` 到 slam-nav 再 `source setup_ros_local.sh`。若未 source，会看到别的机器人大量相机话题（DOMAIN 未隔离）。

---

## 六、与 r1 大学城流程对照

| 项 | r1_pro | Go2（本指南） |
|---|---|---|
| 场景入口 | `scene_daxuecheng.usd` | `scene_daxuecheng_go2.usd` |
| 运动 | swerve `base_controller` | `unitree_rl_lab` 速度策略 |
| bringup | 默认开 controller | 关 swerve，开 Go2 桥 |
| 地图/georef/outdoor | 相同 | 相同（参数 footprint/外参另拷） |

---

*文档版本：2026-09-21；基于 Unitree unitree_rl_lab + unitree_model + Isaac Lab 3.0 + 大学城 slam-nav。  
本机里程碑：Lab play + `--viz kit` 已看到 Go2；大学城场景层 / 传感器 setup / bringup_go2 / cmd_vel 骨架已落地。*
