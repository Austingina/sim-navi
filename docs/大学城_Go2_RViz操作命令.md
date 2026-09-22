# 大学城 Go2 — RViz 前进并回传画面

**环境：** ROS 2 Humble + Isaac Sim 6.0.1 + Unitree-Go2-Velocity（`model_11097` JIT）+ slam-nav + fast_lio2  
**结论：** 可以像轮式一样在 RViz 里发目标让狗前进，并在同一 RViz 里看相机。速度链末端是 **策略跟 `/cmd_vel`**，不是 swerve `/joint_command`。手感比轮式慢、会顿（`max_vx=0.8`）。

验收（2026-09-22）：heading **LOCKED** 后发目标，`/plan`、`/cmd_vel_nav` 有频，约 45 s 跟行约 4.8 m；`/zed/rgb/image_raw` 约 3–4 Hz（640×360 rgb8）。

---

## 和轮式差在哪

| 项 | 轮式 | Go2 |
|---|---|---|
| 场景 / 进程 | `./run_isaacsim.sh --headless scene_daxuecheng.usd` | `./scripts/run_isaac_go2_policy.sh`（策略 + Mid360 + 相机，同进程） |
| bringup | `bringup.launch.py`（含 swerve） | `bringup_go2.launch.py`（**不要**开 `base_controller`） |
| 室外参数 | `start_outdoor.bash -s` → `param_outdoor_sim.yaml`（`max_vx=0.5`） | **必须** `param_outdoor_sim_go2.yaml`（`max_vx=0.8`） |
| 谁吃 `/cmd_vel` | collision_monitor → swerve → `/joint_command` | collision_monitor → **Go2 策略** |
| 画面 | `/zed/rgb/image_raw` | 同一话题；`ISAAC_VIEWPORT=1`（策略脚本默认已开） |

**不要**对 Go2 直接跑 `./start_outdoor.bash -s`：它会加载轮式参数（0.5 m/s），狗跟不住。

---

## 最短成功路径

一键拉起（四个窗口：策略、bringup、室外栈、RViz）：

```bash
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
./scripts/run_go2_rviz_stack.sh
```

要在拉起之后进入交互式定路线，加上 `--route`。后面的名字是默认路线名，省略则是 `route_01`：

```bash
./scripts/run_go2_rviz_stack.sh --route
./scripts/run_go2_rviz_stack.sh --route 北广场一圈
```

已经在跑的那一项会跳过，不会再开一份 Isaac。菜单里怎么确认语义点见下文「语义点串成路线」。

只重启室外导航栈（策略、bringup、RViz 不动）：

```bash
./scripts/run_go2_rviz_stack.sh --nav
```

查询航向是不是 locked（是则退出码 0）：

```bash
./scripts/run_go2_rviz_stack.sh --heading
```

键盘移动。导航栈在跑时发到 `/cmd_vel_smoothed`；没在跑时直接发 `/cmd_vel`。`i` 前进，`,` 后退，`j`/`l` 转向，`k` 或空格停下：

```bash
./scripts/run_go2_rviz_stack.sh --keyboard
```

翻倒后只要这一条。它不重启 Isaac、不重启导航，仿真时间不会回零，`map → base_link` 保持连接，狗留在摔倒的位置被摆正。正在走的目标和语义路线会被取消，避免刚站起来又被带走：

```bash
./scripts/run_go2_rviz_stack.sh --standup
```

终端 1 必须已经是带「扶正=SIGUSR2」的那次策略。摔倒之后不要再 Ctrl+C 终端 1：重启策略会把仿真时间打回 0，RViz 里的狗就会从地图上消失。

回到场景出生点并站好：

```bash
./scripts/run_go2_rviz_stack.sh --origin
```

手开四个终端时：

1. 四个终端：策略 Isaac → `bringup_go2` → outdoor（Go2 yaml）→ RViz
2. 终端 3 出现 `Managed nodes are active`，再等到日志 **`heading LOCKED`**
3. RViz **只用「2D Goal Pose」**，目标在路上且距当前 **>20 m**
4. `ros2 topic hz /plan` 与 `ros2 topic hz /cmd_vel_nav` **必须有频率**
5. 相机：Displays 里 `/zed/rgb/image_raw` 已默认勾选。上方若是纯黑，重开终端 1 后应变淡蓝天空（`setup_sensors_go2` 会加 `/World/sky` 穹顶光；大学城网格本身没有天空）

速度链：

```
campus → Nav2 → /cmd_vel_nav → smoother → /cmd_vel_smoothed
  → collision_monitor → /cmd_vel → Go2 策略 → 关节
```

---

## 启动顺序

先停掉占用 GPU 的旧 Isaac / Lab play（不要和本流程同时开）：

```bash
pkill -9 -f 'isaac_go2_policy_cmd_vel.py' 2>/dev/null || true
pkill -9 -f 'isaacsim.exp.full.kit' 2>/dev/null || true
```

### 终端 1 — Go2 策略（相机默认开）

```bash
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
# 不要 conda activate env_isaaclab；脚本自己清环境
# ISAAC_VIEWPORT 默认 1。只要雷达、不要画面：ISAAC_VIEWPORT=0
./scripts/run_isaac_go2_policy.sh scene_daxuecheng_go2.usd
```

就绪：日志有 `[OK] camera graph: /ActionGraph_camera_go2`，以及周期性 `[cmd_vel] vx=0.00`。

### 终端 2 — bringup（无 swerve，含相机 TF）

```bash
cd ~/navi-sim-code-backups/slam-nav-20260901T135838+0800
source setup_ros_local.sh
source ~/Desktop/fast_lio2/ros2_ws/install/setup.bash
ros2 launch ros2_sensors/bringup_go2.launch.py
```

默认 `with_controller:=false`、`with_camera_tf:=true`（`base_link→zed_link→zed_camera`）。  
**不要** `with_odom_tf:=true`。

### 终端 3 — 室外栈（Go2 慢速参数）

```bash
cd ~/Desktop/fast_lio2/scripts
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
source ~/Desktop/fast_lio2/ros2_ws/install/setup.bash
ros2 launch fast_lio outdoor_bringup_heading_align.launch.py \
  use_sim_time:=true sim:=true enable_elevation:=false \
  nav_params:=$HOME/Desktop/fast_lio2/ros2_ws/src/FAST_LIO/config/param_outdoor_sim_go2.yaml
```

等：Nav2 `active`，日志 **`heading LOCKED and north-aligned`**（`-s` 的 `auto_drive` 由这条 launch 的 `sim:=true` 同样触发，发的是 `/cmd_vel_smoothed`）。

### 终端 4 — RViz（地图 + 画面）

```bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
source ~/Desktop/fast_lio2/ros2_ws/install/setup.bash
rviz2 -d ~/Desktop/fast_lio2/ros2_ws/src/FAST_LIO/rviz/outdoor.rviz \
  --ros-args -p use_sim_time:=true
```

| 项 | 值 |
|---|---|
| Fixed Frame | `map` |
| 发目标 | **只用「2D Goal Pose」** → `/campus_goal_pose` |
| 相机 | Displays：`/zed/rgb/image_raw`（默认已开） |
| 禁用 | 「Nav2 Goal」 |

---

## 发目标前检查

```bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
ros2 lifecycle get /controller_server    # active [3]
ros2 topic echo /heading_align/status --once   # locked
ros2 topic hz /livox/lidar
ros2 topic hz /zed/rgb/image_raw         # 约 3–4 Hz
```

手解锁（仅当迟迟不 LOCKED；outdoor 必须在跑）：

```bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
ros2 topic pub /cmd_vel_smoothed geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}}" -r 10
# 日志 LOCKED 后 Ctrl+C，再发一次空 Twist
ros2 topic pub /cmd_vel_smoothed geometry_msgs/msg/Twist "{}" --once
```

**不要** `ros2 lifecycle set /collision_monitor deactivate`。  
**不要**在终端 3 运行时直接 `pub /cmd_vel`（会被 CM 盖掉；解锁用 `/cmd_vel_smoothed`）。

---

## 发路线

1. heading = **locked**
2. RViz 只点「2D Goal Pose」，目标在路上、距当前 **>20 m**
3. 立刻看频率：

```bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
ros2 topic hz /plan
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel
```

终端 1 的 `[cmd_vel] vx=` 应离开 0（上限 0.8）。狗会走得比轮式慢，中间停一下可以接受；长时间趴地或 `vx` 一直为 0 则没跟上。

有红线 `/campus_path` 不等于在走，必须有 `/plan` + `/cmd_vel_nav`。

### 语义点串成路线再整段发出去

`./scripts/run_go2_rviz_stack.sh --route [路线名]` 拉起四个终端后停在路线菜单里。先确认路线名（直接回车则用参数里的名字，默认 `route_01`）。RViz 用 **2D Goal Pose** 把狗走到位置，终端 3 出现 `Campus route complete` 后，在菜单里输入语义点名字即可确认。黄球话题 `/semantic_route_markers`（已写入 `outdoor.rviz`；脚本拉起前就开着的 RViz 要重开）。

菜单里也可以不走脚本、直接发话题：

1. `heading` = **locked**
2. **2D Goal Pose** 让狗走到要记的位置，日志出现 `Campus route complete` 后再确认
3. 确认当前位置为一个语义点（可重复，按点击顺序串联）：

```bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
ros2 topic pub --once /semantic_route_cmd std_msgs/msg/String "{data: 'mark 北门口'}"
ros2 topic pub --once /semantic_route_cmd std_msgs/msg/String "{data: list}"
```

4. 至少两个点后保存，再整段发出去。狗会按顺序沿路网走到每个点：

```bash
ros2 topic pub --once /semantic_route_cmd std_msgs/msg/String "{data: 'save route_01'}"
ros2 topic pub --once /semantic_route_cmd std_msgs/msg/String "{data: 'run route_01'}"
```

| 命令 | 作用 |
|---|---|
| `mark 名字` | 把狗当前站的位置收成语义点，接到路线末尾。不写名字则是 `p1`、`p2` |
| `undo` / `clear` | 去掉最后一点 / 清空 |
| `list` | 终端 3 打印当前点序 |
| `save 名字` | 写到 `campus_nav/semantic_routes/<名字>.json` |
| `run` | 发当前内存里的路线 |
| `run 名字` | 读文件再发 |
| `stop` | 停下，点还留着 |

跑的过程中再点 **2D Goal Pose** 会暂停这条路线（点不丢），之后 `run` 从头再发。状态可看 `ros2 topic echo /semantic_route_status --once`。

### 误点了「Nav2 Goal」

Humble 的 `ros2 action` 没有 `cancel` 子命令。先在 RViz 工具栏点 **Interact** 退出该工具，再取消已经发出的目标：

```bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```

`return_code: 0` 表示已取消。之后仍只用 **2D Goal Pose**。

---

## 只看画面、不导航

终端 1 起来之后（不必开 outdoor）：

```bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
ros2 topic hz /zed/rgb/image_raw
# 或只开 RViz，看 Image 面板
```

只让狗直行（不跑 Nav2；先停终端 3）：

```bash
source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.8, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 关闭

RViz → 终端 3（Ctrl+C）→ 终端 2 → 终端 1（Ctrl+C）。

规划卡死时只重启室外栈，Isaac 策略可留着：

```bash
pkill -f "outdoor_bringup_heading_align|nav2_container|component_container_isolated|campus_route_planner"
sleep 3
# 再跑终端 3 那条 ros2 launch，重新等到 LOCKED
```

---

## 勿用

| 勿用 | 原因 |
|---|---|
| `./start_outdoor.bash -s` | 加载轮式 `param_outdoor_sim.yaml`（0.5 m/s） |
| `bringup.launch.py` / swerve `base_controller` | Go2 不走 `/joint_command` |
| `./run_isaacsim.sh` 与策略脚本同时开 | 抢 GPU；策略进程已含场景与传感器 |
| RViz「Nav2 Goal」 | 与 campus 跟线冲突；误点后见上文「误点了 Nav2 Goal」 |
| `deactivate collision_monitor` | Nav2 整栈失败 |
| 终端 3 运行时 `pub /cmd_vel` | 解锁请发 `/cmd_vel_smoothed` |
| 不 `source setup_ros_local.sh` | DOMAIN 不对，会看到别的相机话题或没有狗的话题 |
