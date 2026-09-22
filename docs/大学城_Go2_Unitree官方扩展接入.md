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
| `scene_daxuecheng_go2.usd` | 大学城碰撞 + 官方 `go2.usd`；停用 r1；**根路径永久** `active=false` 全部 r1 遗留 `ActionGraph_*` |
| `scripts/isaac_open_stage.py` | Isaac Sim 6 专用：异步 `open_stage` + 自动跑 setup + Play（替代失效的 `openPath`） |
| `ros2_sensors/setup_sensors_go2.py` | 给 `/World/go2/base` 挂 Mid360/IMU/**前置相机** + ROS 图（`*_go2` 专用 ActionGraph；无 swerve） |
| `ros2_sensors/bringup_go2.launch.py` | GPS + `pc2_to_livox`；**默认** `with_controller:=false` |

#### 3.1 一键启动（推荐）

Isaac Sim 6 / Kit 110 下 `--/app/file/openPath` **已失效**，默认还会 `create_new_stage` 出空场景。  
`run_isaacsim.sh --gui scene_daxuecheng_go2.usd` 会自动：

1. `--/isaac/startup/create_new_stage=false`
2. `--exec scripts/isaac_open_stage.py --usd ... --setup setup_sensors_go2.py --play`

```bash
# 终端 A：不要 conda activate env_isaaclab（会污染系统 ros2 → SRE module mismatch）
pkill -9 -f 'isaacsim.exp.full.kit' 2>/dev/null || true
sleep 2
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
source setup_ros_local.sh
./run_isaacsim.sh --gui scene_daxuecheng_go2.usd
```

Kit 日志/终端应依次出现：

```
[isaac_open_stage] open_stage OK: .../scene_daxuecheng_go2.usd
[isaac_open_stage] running setup (sync update): .../setup_sensors_go2.py
[OK] robot graph: /ActionGraph_robot_go2
[isaac_open_stage] setup done
[isaac_open_stage] timeline.play()
```

> `run_isaacsim.sh` 只在 **slam-nav 仓库根**；不在 `~/unitree_rl_lab`。  
> 路径勿带编辑器备份后缀 `scene_daxuecheng_go2.usd~`（会打开空场景）。

可选环境变量：

| 变量 | 默认 | 说明 |
|---|---|---|
| `ISAAC_AUTO_PLAY` | `1` | 设为 `0` 则只 open + setup，不自动 Play |
| `ISAAC_SETUP_SENSORS` | `ros2_sensors/setup_sensors_go2.py` | 覆盖 setup 脚本路径 |
| `ISAAC_VIEWPORT` | GUI 不强制；headless/`run_isaac_go2_policy.sh` 默认 `1` | `1`=渲染并发布 `/zed/*`；`0`=省 GPU（无相机图） |

#### 3.2 ActionGraph 分工（避免 r1 幽灵图）

`scene_daxuecheng_go2.usd` 叠了 `scene_seg_smooth.usd` 子层，其中含 r1 的 `/ActionGraph_robot` 等。  
Go2 传感器使用 **独立路径**，与 r1 图互不覆盖：

| 路径 | 来源 | 状态 |
|---|---|---|
| `/ActionGraph_control` `/ActionGraph_camera` `/ActionGraph_robot` `/ActionGraph_lidar` `/ActionGraph_imu` | r1 子层 | USD 层内 **`active=false`**（必须；仅靠运行时 `SetActive` 不够，Play 仍会 segfault） |
| `/ActionGraph_robot_go2` `/ActionGraph_lidar_go2` `/ActionGraph_imu_go2` `/ActionGraph_camera_go2` | `setup_sensors_go2.py` 创建 | Play 后发布 ROS 话题（含 `/zed/*`） |

IMU 用 `IsaacImuSensor.Define`（不依赖易未注册的 `IsaacSensorCreateImuSensor` 命令）。

#### 3.3 手动 fallback

若自动 setup 失败，GUI 已打开场景时：

1. **Window → Script Editor** → 打开并 Run `ros2_sensors/setup_sensors_go2.py`
2. 点 **Play**

#### 3.4 贴地与验收

贴地：路面约 `z≈-0.42`；层内默认 `go2` 平移 `(0,0,0.03)`。若穿地/悬空，只改 `scene_daxuecheng_go2.usd` 里 `xformOp:translate` 的 z。

**Stage 验收：**

- 有 `/World/go2/base`；根上 r1 的 `/ActionGraph_*` 为 **inactive**
- 有 `/ActionGraph_robot_go2` 等 `*_go2` 图（setup 成功后）

**ROS 验收（终端 B，须先 `source setup_ros_local.sh`）：**

```bash
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
source setup_ros_local.sh
ros2 topic hz /clock
ros2 topic hz /livox/lidar_raw
ros2 topic hz /livox/imu
ros2 topic hz /zed/rgb/image_raw   # GUI 或 ISAAC_VIEWPORT=1
```

Play 后应见 `/clock /odom_gt /livox/lidar_raw /livox/imu /zed/rgb/image_raw /zed/camera_info /tf /tf_static`。

> 仅加载 USD **不会走路**。走路靠步骤 4 的速度策略。  
> 相机：官方 Go2 无 `zed_link`，`setup_sensors_go2` 在 `base` 下自建 `zed_link/zed_camera`，话题与 r1 对齐 `/zed/*`（`outdoor.rviz` Image 面板可直接用）。

### 步骤 4 — `/cmd_vel` → Unitree-Go2-Velocity

`base_velocity` 指令与 `/cmd_vel` 同语义（机体 `vx, vy, wz`）。

| 方案 | 说明 | 本仓库状态 |
|---|---|---|
| **A. Lab play + `/cmd_vel`** | 默认平地 + 策略；ROS 写 `vel_command_b` | **指令通路已验收**；权重加长训见 4.1 |
| **A2. Lab + 大学城地形** | `terrain_type=usd` + `--campus_terrain` | **已验收**（阶段 B，2026-09-22） |
| **B. Isaac 场景 + 外挂策略** | `scene_daxuecheng_go2` + JIT 策略 | **已验收**（阶段 C，2026-09-22） |

冒烟（Lab 平地，确认指令接通）：

```bash
# 勿与 Isaac GUI 大学城场景抢 GPU：先停 kit
pkill -9 -f 'isaacsim.exp.full.kit' 2>/dev/null || true

# env_isaaclab（Python 3.12）通常无 Humble rclpy；脚本会自动起 /usr/bin/python3 中继
source ~/miniforge3/etc/profile.d/conda.sh
conda activate env_isaaclab
export ISAACSIM_PATH=~/isaacsim/current ISAAC_PATH=$ISAACSIM_PATH
export CARB_APP_PATH=$ISAACSIM_PATH/kit EXP_PATH=$ISAACSIM_PATH/apps
export DISPLAY=:1
source $ISAACSIM_PATH/setup_python_env.sh   # Sim 6 二进制无 setup_conda_env.sh，必须这一步
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1

cd ~/unitree_rl_lab
python ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/scripts/lab_play_go2_cmd_vel.py \
  --task Unitree-Go2-Velocity --viz kit --num_envs 1

# 另一终端（系统 bash，不要 conda）
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
source setup_ros_local.sh
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

脚本日志应周期性打印 `[cmd_vel] vx=0.30 ... pos=(x,y,z)`（无指令时为 `vx=0.00`）。

**本机已验收（2026-09-22）：**

1. 短训 `model_99`：仅指令日志变化，站不稳。  
2. resume `model_3098`（run `2026-09-22_10-23-42`）：发 `vx=0.3` 约 15s，`z≈0.32` 站稳并有位移；停发后指令清零。  
3. **肉眼观感（重要）：** 能走，但**不顺畅**——多数时间像站着/小幅蹭步，连续跟速不如 r1 轮式稳。阶段 A 只证明「能动」，**不等于**导航可用的稳走。  
4. **A+（`model_11097`，run `2026-09-22_11-04-32`）：** 连续 `pub -r 10 vx=0.3` 约 30s，跟速段位移步 **100%**，`z` 稳在 ~0.32–0.33；停发后能站稳。仍不如 r1 轮式顺滑，但已达进 B 门槛。

> **阶段 A / A+ 均已通过。** 可进阶段 B（校园 terrain），验收标准见「B 质量门槛」——**不要求达到轮式稳定**。
### 步骤 4.1 — 真正能走（计划与进度）

目标：策略稳走 + `/cmd_vel` 驱动关节；最终在大学城地形/场景里跟 outdoor 导航。

```
阶段 A   Lab 平地加长训 → 冒烟：站稳 + 有位移
阶段 A+  继续训到「连续跟 /cmd_vel」较顺（仍达不到轮式，但少站着发呆）
   │
阶段 B   Lab 地形换大学城 → play 在校园 mesh 上走（预期更抖，勿对标轮式）
   │
阶段 C   （可选）Isaac scene + 外挂策略
   │
阶段 D   outdoor 跟线（heading LOCKED）
```

| 阶段 | 内容 | 关键文件 / 命令 | 状态 |
|---|---|---|---|
| **A** | resume ≥3000；play 有位移 | `model_3098.pt` | **冒烟通过**（走得不顺） |
| **A+** | 从 `model_3098` 再 resume → `model_11097`；连续 `-r 10` 跟速 | run `2026-09-22_11-04-32` / `model_11097.pt` | **通过**（2026-09-22：30s `vx=0.3`，跟速段移动步 100%，`z≈0.32`，path~连续；仍不如轮式） |
| **B** | 大学城 terrain；能挪动 + **雷达叠影/障碍误判可接受**；勿对标轮式 | `--campus_terrain` + `lidar_ghost_check.py` | **通过（有条件）** 2026-09-22：见下方验收 |
| **C** | Isaac 外挂策略：`/cmd_vel`→关节 + Mid360 同进程 | `scripts/isaac_go2_policy_cmd_vel.py` + `run_isaac_go2_policy.sh` | **通过**（2026-09-22：走 5.2m；行走 `jaccard≈0.64`） |
| **D** | outdoor 跟线 | `param_outdoor_sim_go2.yaml` + `start_outdoor -s` | **通过**（2026-09-22：LOCKED；发目标后 `/plan`≈0.8Hz `/cmd_vel_nav`≈5Hz；跟速约 4.8m） |

**与轮式（r1）对比 — 做 B / D 前必读：**

| | r1 swerve | Go2 RL（当前） |
|---|---|---|
| 执行器 | 轮速近似一阶跟 `/cmd_vel` | 策略→关节，延迟大、易停顿 |
| 观感 | 连续平滑 | 易「走两步站一会」 |
| 机身姿态 | 俯仰/横滚小，雷达外参近似刚体平移 | 踏步抖动大 → 雷达射线帧间不重合，易**叠影** |
| 感知后果 | costmap / 地面分割较稳 | 障碍物「重影」；平地被当成障碍，或障碍被抹平 |
| 大学城地形 | 碰撞+控制已调多年 | 换 mesh 后更易绊/滑/重置 |
| 阶段 B 目标 | — | **能在校园 mesh 上跟速挪动、少摔**；**并核对雷达叠影是否误伤导航**；**不追求**与轮式同等稳定 |
| 阶段 D 目标 | 路网跟线 | 先能 LOCKED + 慢速跟线；卡顿可接受，趴地不可接受；**若叠影导致 costmap 乱墙，先降速/加滤波再跟线** |

**阶段 A+ 通过标准（进 B 前）：**

1. 持续 `pub /cmd_vel linear.x:=0.3 -r 10` ≥20s：`z` 稳定在 ~0.3，**位移连续**（不是偶发挪一下）。  
2. 停发后站稳，不乱抖。  
3. 肉眼：跟速时段「站着发呆」明显减少（仍允许偶发顿挫）。  
4. **明确不要求**达到 r1 轮式的顺滑度。

**阶段 B 质量门槛（进 D 前）：**

1. 校园 mesh 上同样跟 `vx=0.3` 能挪动，摔倒率可接受（偶发 reset OK）。  
2. 记录与平地差距（更抖 / 更易停是预期）。  
3. 若平地 A+ 都跟不好，**禁止**用全量 `daxuecheng-collision.usdz` 开训/play 硬上——先裁小区域 mesh 或继续 A+。  
4. **雷达 / 感知连带验收（不如轮式稳时必查）：**  
   - 站立静止：RViz 看 `/livox/lidar`（或 `points`）墙面/柱子应清晰，无明显「双影」。  
   - 跟速行走：同一障碍是否出现**叠影 / 拖影**；地面点是否高低跳、把平地扫成「台阶」。  
   - 对照 costmap / 障碍层：是否误出幽灵墙、或真实障碍被冲淡。  
   - 若叠影严重：优先 **降速**（如 `vx≤0.2`）、核对 IMU→雷达外参与时间戳、必要时对点云做运动补偿/更强体素滤波；**不要**假定「能挪动 = 导航感知可用」。  
5. B 通过 = **能挪动 + 感知误判可接受（或已有对策）**；仅能走但 costmap 满屏鬼影 → **未过 B，勿进 D**。

**本机阶段 B 验收（2026-09-22）：**

1. **Lab 大学城 terrain 可挪动：** `lab_play_go2_cmd_vel.py --campus_terrain` + `model_11097`；terrain=`daxuecheng-collision-smooth.usdz`，spawn `(0,2,-0.014)`。连续 `vx=0.3 -r 10` ≈35s：移动步 **100%**，`path_xy≈7m`，`z≈-0.09`（偶发 episode reset 可接受）。  
2. **行走叠影代理（Lab，无 Mid360）：** 跟速段 `ang_xy_mean≈0.19`（LOW）、`hscan_std_mean≈0.003`（平地扫描稳定）→ 机身抖动不足以把平地扫成明显「台阶」。  
3. **Mid360 静止基线（Isaac `scene_daxuecheng_go2` headless + setup_sensors_go2）：** `scripts/lidar_ghost_check.py --topic /livox/points` → `jaccard_mean=1.0`，`VERDICT=OK_LIKE_STATIC`。  
4. **Mid360 行走叠影：** 已由阶段 C 同进程验收（见下）；进 D 仍建议 `vx≤0.2` 起。

**本机阶段 C 验收（2026-09-22）：**

1. 导出 JIT：`scripts/export_go2_policy_jit.py` → `.../2026-09-22_11-04-32/exported/policy.pt`  
2. 启动：`./scripts/run_isaac_go2_policy.sh scene_daxuecheng_go2.usd`  
3. **能挪：** `vx=0.3 -r 10` ≈30s，移动步 100%，`path_xy≈5.2m`，`z≈-0.10`  
4. **行走 lidar_ghost_check：** `jaccard_mean≈0.64`（静止≈1.0），整体可接受；`jaccard_p10≈0.18` 偶发更糊  

```bash
./scripts/run_isaac_go2_policy.sh scene_daxuecheng_go2.usd
# 另终端
source setup_ros_local.sh
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
python3 scripts/lidar_ghost_check.py --topic /livox/points --seconds 20
```

**阶段 B play 命令：**

```bash
# 干净环境（勿污染 conda 的 Isaac PYTHONPATH）
env -i HOME=$HOME USER=$USER PATH=/usr/bin:/bin:$HOME/miniforge3/bin DISPLAY=:1 \
  bash -lc '
  source ~/miniforge3/etc/profile.d/conda.sh && conda activate env_isaaclab
  export ISAACSIM_PATH=~/isaacsim/current ISAAC_PATH=$ISAACSIM_PATH
  export CARB_APP_PATH=$ISAACSIM_PATH/kit EXP_PATH=$ISAACSIM_PATH/apps
  source $ISAACSIM_PATH/setup_python_env.sh
  cd ~/unitree_rl_lab
  python ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/scripts/lab_play_go2_cmd_vel.py \
    --task Unitree-Go2-Velocity --num_envs 1 --campus_terrain \
    --checkpoint logs/rsl_rl/unitree_go2_velocity/2026-09-22_11-04-32/model_11097.pt
'
# 另终端 -r 10 发 /cmd_vel（同 ROS_DOMAIN_ID=7）
```

**雷达叠影检查：**

```bash
./run_isaacsim.sh --headless scene_daxuecheng_go2.usd   # 已自动 setup_sensors_go2
source setup_ros_local.sh
python3 scripts/lidar_ghost_check.py --topic /livox/points --seconds 20
```

**阶段 A / A+ — resume 命令：**

```bash
pkill -9 -f 'isaacsim.exp.full.kit' 2>/dev/null || true
source ~/miniforge3/etc/profile.d/conda.sh && conda activate env_isaaclab
export ISAACSIM_PATH=~/isaacsim/current ISAAC_PATH=$ISAACSIM_PATH
export CARB_APP_PATH=$ISAACSIM_PATH/kit EXP_PATH=$ISAACSIM_PATH/apps
source $ISAACSIM_PATH/setup_python_env.sh
cd ~/unitree_rl_lab

# A（已完成）：从 model_99 → 3000
# A+（建议）：从 model_3098 再加长
./unitree_rl_lab.sh -t --task Unitree-Go2-Velocity \
  --resume \
  --load_run 2026-09-22_10-23-42 \
  --checkpoint model_3098.pt \
  --max_iterations 8000 \
  --num_envs 1024
```

**play 验收（连续跟速，勿只发一次）：**

```bash
python .../lab_play_go2_cmd_vel.py --task Unitree-Go2-Velocity --viz kit --num_envs 1 \
  --checkpoint logs/rsl_rl/unitree_go2_velocity/<run>/model_XXXX.pt
# 另终端必须 -r 持续发，否则 cmd_timeout 会清零 → 看起来「一直站着」
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**阶段 B 技术要点（勿直接全量 usdz × 4096 env）：**

1. `terrain_type="usd"` + 裁剪小区域 collision，或先 `"plane"` 冒烟  
2. `CurriculumCfg.terrain_levels = None`；play 勿改 `terrain_generator`  
3. init z 对齐大学城路面；`height_scanner` 指向真实地面  
4. 验收时对照「B 质量门槛」：能挪动 + **雷达叠影/障碍误判可接受**；**不要用轮式手感当失败标准**，但也**不能忽略感知副作用**

**不做 C 的条件：** B 在校园 mesh 上已能跟 `/cmd_vel` 挪动，且 outdoor 走 Lab 主循环即可。

### 步骤 5 — ROS 2 bringup（对接 outdoor 栈）

| 话题 | 方向 | 来源 |
|---|---|---|
| `/cmd_vel` | Go2/Lab 订阅 | Nav2 CM 输出；**勿**开 swerve `base_controller` |
| `/livox/lidar_raw` | Isaac 发布 | `setup_sensors_go2` |
| `/livox/lidar` + `/livox/points` | bringup | `pc2_to_livox.py` |
| `/livox/imu` | Isaac 发布 | 与雷达同 `livox_frame` |
| `/zed/rgb/image_raw` `/zed/camera_info` | Isaac 发布 | `setup_sensors_go2` 相机图；需 GUI 或 `ISAAC_VIEWPORT=1` |
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
# Go2 慢速参数（本机已生成）
cd ~/Desktop/fast_lio2/scripts
ros2 launch fast_lio outdoor_bringup_heading_align.launch.py \
  use_sim_time:=true sim:=true enable_elevation:=false \
  nav_params:=$HOME/Desktop/fast_lio2/ros2_ws/src/FAST_LIO/config/param_outdoor_sim_go2.yaml
# 或改 start_outdoor.bash -s 里的 NAV_PARAMS_ARG 指向上述 go2 yaml
```

**本机阶段 D 验收（2026-09-22）：**

1. Isaac：`./scripts/run_isaac_go2_policy.sh scene_daxuecheng_go2.usd`  
2. bringup：`ros2 launch ros2_sensors/bringup_go2.launch.py`  
3. outdoor：上式 + Go2 `nav_params` → 日志 **`heading LOCKED`**；Nav2 `active`  
4. 发 `/campus_goal_pose`（例 map≈3375,2310）→ `/plan`≈0.8 Hz、`/cmd_vel_nav`≈5 Hz、`/cmd_vel`≈20 Hz；策略侧约 45s 跟行 `path_xy≈4.8m`  

### 步骤 7 — 端到端（大学城导航）

可复制命令见 [大学城_Go2_RViz操作命令.md](大学城_Go2_RViz操作命令.md)。与轮式指南相同的是 RViz「2D Goal Pose」和 `/zed/rgb/image_raw`；**不要**用 `start_outdoor.bash -s`（那是轮式 0.5 m/s 参数）。要点：

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
| `scripts/isaac_open_stage.py` | Sim 6 GUI：异步 open + sync setup + 可选 Play |
| `scripts/lab_play_go2_cmd_vel.py` | Lab 播放 + `/cmd_vel`→`base_velocity`（无 rclpy 时自动系统 Python 中继） |
| `scripts/_cmd_vel_relay_helper.py` | 中继辅助（Humble `/usr/bin/python3`） |
| `third_party/unitree_model/Go2/` | 官方模型（大文件，勿强行 git 提交） |
| `scene_daxuecheng_go2.usd` | 大学城 + Go2；停用 r1；根路径 r1 ActionGraph 永久 inactive |
| `ros2_sensors/setup_sensors_go2.py` | Go2 Mid360/IMU/前置相机/ROS（`*_go2` 图；禁止 `app.update()`） |
| `ros2_sensors/bringup_go2.launch.py` | 无 swerve 的 outdoor bringup（`/usr/bin/python3`） |
| `ros2_sensors/go2_topic_bridge.py` | 非标准命名空间话题重映射 |
| `setup_ros_local.sh` | ROS_DOMAIN_ID=7 + `conda deactivate` + 系统 python 置顶 |
| `docs/大学城_Go2_Unitree官方扩展接入.md` | 本文 |

**本机另改（不在本仓库）：** `~/unitree_rl_lab` 的 Lab3 补丁与 `unitree_rl_lab.sh -p --viz kit`。

---

## 四、验收清单

- [x] `go2.usd` 已下载
- [x] `unitree_rl_lab` 能列出 / train / play `Unitree-Go2-Velocity`（Kit 视口可见 Go2）
- [x] `scene_daxuecheng_go2.usd`：Go2 足底约 z=-0.42；根 r1 `ActionGraph_*` 永久 inactive
- [x] `run_isaacsim.sh` + `isaac_open_stage.py` 自动 open/setup/play（Sim 6 空场景问题已规避）
- [x] GUI 跑通 `bringup_go2`（`/livox/lidar`≈8 Hz、`/livox/imu`≈48 Hz；2026-09-22 本机验收）
- [x] `lab_play_go2_cmd_vel.py` 下 `/cmd_vel` 非零时策略指令日志变化（2026-09-22：`vx=0.00`→`vx=0.30`；狗移动视权重，当前 model_99 短训可能仍不稳）
- [x] outdoor Nav2 `active`；发目标后 `/plan`≈0.8–5 Hz、`/cmd_vel_nav`≈8–20 Hz（2026-09-22）
- [x] **能走 A：** resume ≥3000；play 有位移（**走得不顺，多数时间像站着**——见 4.1 观感说明）
- [x] **能走 A+：** `model_11097`；连续 `-r 10` 跟速段移动 100%、`z≈0.32`（2026-09-22）
- [x] **能走 B：** Lab 大学城 terrain 可挪动（`model_11097`+`--campus_terrain`，2026-09-22）；行走叠影代理 LOW + Mid360 静止 `jaccard=1.0`
- [x] **能走 C：** Isaac 内 `/cmd_vel`→策略能挪 + 行走 `lidar_ghost_check` `jaccard≈0.64`（2026-09-22）
- [x] **能走 D：** outdoor LOCKED + 慢速跟线（`param_outdoor_sim_go2` max_vx=0.2；发 `/campus_goal_pose` 后 `/plan`/`/cmd_vel_nav` 有频，跟约 4.8m；2026-09-22）

> **RViz / 画面说明：** 可用 `outdoor.rviz` 看地图、路网路径、点云、跟线；**相机面板**订 `/zed/rgb/image_raw`（与轮式 r1 同话题）。  
> Go2 相机已由 `setup_sensors_go2` 默认挂载：`/World/go2/base/zed_link/zed_camera` → `/ActionGraph_camera_go2`。GUI 自动有图；headless / 策略进程需 `ISAAC_VIEWPORT=1`（`run_isaac_go2_policy.sh` 已默认）。  
> 验收参考（2026-09-22）：`/zed/rgb/image_raw` ≈3–4 Hz（640×360 rgb8，`frame_id=zed_camera`）。`base_link→zed_link→zed_camera` 静态 TF 由 `bringup_go2`（`with_camera_tf:=true`）发布。

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

### Q8：Isaac 打开是空 New Stage / 启动后 segfault / `tzfile` assert？

**空场景：** Isaac Sim 6 默认 `create_new_stage=true`，且 `openPath` 无效。必须用 `./run_isaacsim.sh --gui scene_daxuecheng_go2.usd`（内部走 `isaac_open_stage.py`）。勿手写 `pkill ill` 之类截断命令。

**segfault（常见链）：**

1. 在 `--exec` 的 **async 协程里同步跑 setup**（`og.Controller` / `app.update()`）→ 事件循环死锁 → Play 崩溃。  
   现版 `isaac_open_stage.py` 已改为：**async 只 open + 等扩展；setup/play 在 sync update 回调**；`setup_sensors_go2.py` **禁止** `app.update()`。
2. Play 后日志仍报 `/ActionGraph_robot` 找 `/World/r1_pro_with_gripper/...` → r1 **幽灵图仍在跑**。  
   须在 `scene_daxuecheng_go2.usd` 对 `ActionGraph_robot/lidar/imu` 写 **`active=false`**（Go2 用 `*_go2` 图，不冲突）。仅运行时 `SetActive(False)` 不够。
3. 终端末尾 `kit: tzfile.c:... Assertion failed` 多为 **#1/#2 崩溃后的连带噪音**，不必单独修时区。

**排查顺序：**

```bash
pkill -9 -f 'isaacsim.exp.full.kit' 2>/dev/null || true
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
source setup_ros_local.sh
./run_isaacsim.sh --gui scene_daxuecheng_go2.usd
# 看日志：setup done → timeline.play()，且不应再出现 ActionGraph_robot + r1_pro
```

仍失败时：`ISAAC_AUTO_PLAY=0 ./run_isaacsim.sh --gui scene_daxuecheng_go2.usd`，手动 Script Editor 跑 setup 再 Play。

### Q9：有 `/clock`、`/livox/lidar_raw` 但没有 `/livox/imu`？

1. 确认 Play 已按，且 Stage 有 `/World/go2/base/imu_livox`（`IsaacImuSensor`）。  
2. 确认 `/ActionGraph_imu_go2` 存在且 active；根路径 r1 的 `/ActionGraph_imu` 应为 inactive。  
3. 重跑 setup：关闭 Isaac → 重新 `./run_isaacsim.sh --gui scene_daxuecheng_go2.usd`（会自动 setup）。  
4. `ros2 topic hz /livox/imu` 前必须 `source setup_ros_local.sh`（DOMAIN_ID=7）。

### Q10：bringup 报 `No module named 'numpy'` / `rclpy._rclpy_pybind11`？

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

*文档版本：2026-09-22；基于 Unitree unitree_rl_lab + unitree_model + Isaac Lab 3.0 + Isaac Sim 6.0.1 + 大学城 slam-nav。  
本机里程碑：传感器/bringup/Nav2/`/cmd_vel` 指令通路已验收；**真正能走**按步骤 4.1：A 加长训进行中 → B 大学城地形 → D outdoor 跟线。*
