# Go2 大学城场景固定站立测试操作记录

- 操作时间：2026-09-21T21:56:13+08:00
- 工作目录：`/home/ai-sz-a26394-u1/navi-sim-code-backups/slam-nav-20260901T135838+0800`
- Git 分支/基线：`isaac-sim` / `38db451`
- 操作类型：本机 Isaac Sim 仿真；不访问实体机器人，不发送 ROS 运动指令

## 目标

在 `scene_daxuecheng_go2.usd` 内对 `/World/go2/base` articulation 连续施加零速度的固定关节位置目标，验证 Go2 在重力和大学城地面碰撞下能否保持名义站姿。该测试只隔离模型、碰撞、初始高度、关节顺序和静态 PD 控制，不加载或评价运动策略。

## 当前已确认基线

1. `run_isaacsim.sh --gui scene_daxuecheng_go2.usd` 只负责打开场景、运行 `ros2_sensors/setup_sensors_go2.py`、自动 Play。
2. `setup_sensors_go2.py` 只创建传感器与 ROS 2 OmniGraph，不写关节位置/力矩。
3. 当前相关文件 SHA-256：
   - `run_isaacsim.sh`: `e9cf2f06d19b37837689680dfbd86cd5ee59281b8733d71bf0424d63bbec17e3`
   - `ros2_sensors/setup_sensors_go2.py`: `814734391865832f0215762d03cf7a6ed0b2b648850aad1bfe4332b9432bc507`
   - `scene_daxuecheng_go2.usd`: `0a8e79aeb2d0c7424e013ef91536959fead726ce4a307515bb1d43167374f4ef`
4. Unitree Go2 工程名义站姿/PD 基线：根高度约 `0.4 m`；左右髋 `-0.1/+0.1 rad`；前腿大腿 `0.8 rad`；后腿大腿 `1.0 rad`；小腿 `-1.5 rad`；`Kp=25`、`Kd=0.5`。

## 变更范围

计划新增一个独立、可回滚的测试脚本，不直接修改场景 USD、传感器脚本或导航/ROS 配置。脚本应：

1. 无头加载大学城 Go2 场景；
2. 初始化 articulation 并打印真实 DOF 名称和顺序；
3. 按关节名映射名义站姿，拒绝未知/缺失的 12 个腿关节；
4. 在 Play 前设置初始关节角和零速度；
5. 连续施加 `Kp=25`、`Kd=0.5` 的位置保持目标；
6. 运行有界测试，定期记录 base 高度、横滚/俯仰、关节误差；
7. 以明确阈值输出 PASS/FAIL，并自动退出 Isaac Sim。

## 保持不动的内容

- `scene_daxuecheng_go2.usd` 本体；
- `run_isaacsim.sh`；
- `ros2_sensors/setup_sensors_go2.py`；
- ROS 2 Domain/RMW 配置；
- Isaac Lab 训练代码、checkpoint 和实体机器人。

## 计划命令

```bash
# 静态检查
python3 -m py_compile scripts/go2_static_stand_test.py

# 有界无头测试（预计 10 秒仿真时间）
timeout 180s ~/isaacsim/current/python.sh scripts/go2_static_stand_test.py \
  --usd "$PWD/scene_daxuecheng_go2.usd" --duration 10
```

## 成功判据

- 识别到且只控制 12 个 Go2 腿关节；
- 测试进程在超时前自行退出，退出码 0；
- 最后 2 秒内 base 高度不低于 `0.22 m`；
- 最后 2 秒内 `|roll|`、`|pitch|` 不超过 `35°`；
- 关节位置目标持续写入，无 NaN/异常；
- 输出至少包含初始、周期采样和最终统计。

## 失败出口

遇到下列任一情况立即失败并保留日志：场景/prim 缺失、DOF 名称不匹配、物理初始化失败、base 高度跌破阈值、姿态翻倒、进程崩溃或超时。失败后不通过“加大力矩直到站住”掩盖问题；先定位根高度、碰撞、关节符号/顺序、drive 类型与力矩限制。

## 回滚

删除新增测试脚本和本记录即可；本操作不应改写 USD 或现有启动脚本。若测试意外产生 USD 修改，立即以操作前 SHA-256 为基线核对并恢复。

## 实际执行记录

- 完成时间：2026-09-21T22:06:47+08:00
- 新增脚本：`scripts/go2_static_stand_test.py`
- 最终命令：

```bash
python3 -m py_compile scripts/go2_static_stand_test.py
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u PYTHONPATH -u PYTHONHOME \
  timeout 240s "$HOME/isaacsim/current/python.sh" \
  scripts/go2_static_stand_test.py \
  --usd "$PWD/scene_daxuecheng_go2.usd" --duration 10
```

### 执行中发现与计划偏差

1. 原计划用绝对 `base z >= 0.22 m` 判站立，这在本场景不成立。`scene_daxuecheng_go2.usd` 明确记录大学城路面约 `z=-0.42 m`，并把 `/World/go2` 的 authored translation 设为 `z=0.03 m`；因此最终 base 世界坐标为负值不等于塌腿。
2. 判据改为与地形高程无关的组合：最后 2 秒 `base` 到最低足端的垂向净高、base 高度波动、roll/pitch。保留原计划内容，不把修订后的判据伪装成事前预期。
3. 试过一次 `--spawn-height 0.4` 诊断覆盖，机器人仍能保持站姿，但这不是场景原始出生位。最终验收取消覆盖，严格使用场景 authored pose。
4. Isaac `SimulationApp.close()` 使外层 `python.sh` 即使脚本打印 FAIL 也可能返回 0；最终包装命令同时检查明确的 `[PASS]` 标记，不能只看进程退出码。

### 中间失败证据

- `run_20260921T215837+0800.log`：旧的绝对高度判据误判失败；实际姿态没有翻倒。
- `run_20260921T220148+0800_spawn040.log`：同样因绝对高度判据误判失败。
- `run_20260921T220512+0800_clearance.log`：首次读取 body 名称时误用 `Robot.body_names`，触发 `AttributeError`；已改用 articulation view 的 body 元数据。

### 最终结果

最终日志：`logs/go2_static_stand/run_20260921T220632+0800_stage_pose_final.log`

- 真实 DOF：12 个，关节名完整匹配 Go2 四腿髋/大腿/小腿；
- 真实刚体：19 个，识别到 `FL/FR/RL/RR_foot` 四个足端；
- 场景 authored/reset base：`z=0.0292426 m`，未做出生高度覆盖；
- 仿真时长：10.000 s，200 Hz，连续静态 PD（`Kp=25`、`Kd=0.5`）；
- 最后 2 秒最小 base 世界高度：`-0.1510 m`（受大学城负高程基准影响，不单独用于判定）；
- 最后 2 秒 base 高度波动：`0.0003 m`；
- 最小 base-最低足端垂向净高：`0.2570 m`；
- 最大 roll/pitch 倾角：`1.35°`；
- 10 秒末姿态：roll `-1.23°`，pitch `1.32°`；
- 最终标记：`[PASS] Go2 held the nominal standing pose under continuous static PD control`；
- 包装验收：`marker_verdict=0`。

### 文件与完整性

- `scripts/go2_static_stand_test.py` SHA-256：`7f43e16b64b22f17e60df9eda0d81501e37a7de064aa3f9a87eca76bcf321cd5`；
- 最终日志 SHA-256：`e0083ae94a05e2cb5c0049ca51a09fbb8deaedd18640d8dc3bdb3d7380fd5ad6`；
- 场景及现有入口在操作后保持原 SHA-256：
  - `scene_daxuecheng_go2.usd`: `0a8e79aeb2d0c7424e013ef91536959fead726ce4a307515bb1d43167374f4ef`
  - `run_isaacsim.sh`: `e9cf2f06d19b37837689680dfbd86cd5ee59281b8733d71bf0424d63bbec17e3`
  - `ros2_sensors/setup_sensors_go2.py`: `814734391865832f0215762d03cf7a6ed0b2b648850aad1bfe4332b9432bc507`

## 结论

固定站立测试通过：同一大学城碰撞场景、同一 Go2 模型，在不加载运动策略的情况下，只要持续施加名义关节位置和静态 PD，Go2 可稳定保持直立。由此可把“模型/碰撞完全无法站立”排除；当前 GUI 流程倒地的直接原因仍是没有持续关节控制。该结果只证明静态站立，不证明 locomotion policy 已可用或已接入大学城场景。
