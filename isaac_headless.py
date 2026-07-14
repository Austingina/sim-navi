#!/usr/bin/env python3
"""
Isaac Sim 纯无头运行器（standalone）——不开渲染窗口、自动 Play、只走 ROS 2。

给 run_isaacsim.sh --headless 调用（用 isim/python.sh 执行，环境已由外层脚本清理干净）。
适合本机"太卡"时：不渲染视口、GPU 负担最低；场景里的 OmniGraph（/clock、/odom_gt、
/tf、/livox/* 等）随 timeline 播放正常发布，你在别的机器/终端用 rviz、FAST-LIO 订阅即可。

用法（一般不直接调，用 run_isaacsim.sh --headless）：
    isim/python.sh isaac_headless.py /abs/path/scene.usd [--/kit/settings=...]

Ctrl+C 退出。
"""
import sys

# 取第一个不以 -- 开头的参数当场景路径；其余(--/...)留给 kit 解析。
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
USD_PATH = _args[0] if _args else ""

# SimulationApp 必须在导入任何 omni.* 之前创建。headless=True: 不开窗口。
import os as _os  # noqa: E402
from isaacsim import SimulationApp  # noqa: E402

# 关键性能开关：SLAM 不需要渲染(PhysX 雷达/IMU 都不吃 RTX)。
# disable_viewport_updates=True -> 不再每帧渲染重场景(实测 25ms/帧 -> ~物理级),
# 但 OmniGraph 仍评估、/clock /livox/* /tf 照常发布。
# 需要相机图像时设 ISAAC_VIEWPORT=1 恢复渲染。
_viewport = _os.environ.get("ISAAC_VIEWPORT", "0") == "1"
simulation_app = SimulationApp({"headless": True, "disable_viewport_updates": not _viewport})
print(f"[headless] disable_viewport_updates={not _viewport} "
      f"(ISAAC_VIEWPORT={_os.environ.get('ISAAC_VIEWPORT','0')}; =1 可恢复渲染/相机)")

import carb  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import open_stage, is_stage_loading  # noqa: E402


def report_timestep():
    """打印时间轴步频(timeCodesPerSecond)和物理步频(1/dt)。无头/GUI 都能用。"""
    from pxr import UsdPhysics, PhysxSchema  # noqa: PLC0415
    stage = omni.usd.get_context().get_stage()
    tcps = stage.GetTimeCodesPerSecond()
    print(f"[dt] timeCodesPerSecond(时间轴步频) = {tcps}  -> 时间轴 dt = {1.0 / tcps:.5f}s"
          if tcps else "[dt] timeCodesPerSecond 未设置")
    found = False
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene):
            found = True
            api = PhysxSchema.PhysxSceneAPI.Get(stage, p.GetPath())
            attr = api.GetTimeStepsPerSecondAttr()
            v = attr.Get() if attr else None
            if v:
                print(f"[dt] PhysicsScene {p.GetPath()}  TimeStepsPerSecond = {v}"
                      f"  -> 物理 dt = {1.0 / v:.5f}s")
            else:
                print(f"[dt] PhysicsScene {p.GetPath()}  TimeStepsPerSecond 未显式设置"
                      f" -> 用默认(通常 60 -> dt=0.01667s)")
    if not found:
        print("[dt] 场景里没有显式 PhysicsScene -> 运行时按默认 60 步/秒(dt=0.01667s)")

# 场景里固化的 OmniGraph 用到这些扩展的节点类型，无头下要显式确保开启，
# 否则图不评估、话题不发。
for _ext in (
    "omni.graph.action",             # OnPlaybackTick
    "isaacsim.core.nodes",           # IsaacReadSimulationTime / IsaacComputeOdometry
    "isaacsim.sensors.physx",        # IsaacReadLidarPointCloud (PhysX 雷达)
    "isaacsim.sensors.physics",      # IsaacReadIMU
    "isaacsim.ros2.bridge",          # 所有 ROS2Publish* + TF
):
    try:
        enable_extension(_ext)
    except Exception as e:  # noqa: BLE001
        carb.log_warn(f"[headless] enable_extension({_ext}) 失败: {e}")
simulation_app.update()

if not USD_PATH:
    carb.log_error("[headless] 未提供场景 .usd 路径，无事可做。退出。")
    simulation_app.close()
    sys.exit(1)

print(f"[headless] 打开场景: {USD_PATH}")
open_stage(USD_PATH)
# 等资源加载完再 Play，避免图/物理还没就绪
while is_stage_loading():
    simulation_app.update()

# ISAAC_VIEWPORT=0 表示本次不要相机数据。仅 disable_viewport_updates 还不够：
# ActionGraph_camera 创建的 Replicator render products 仍会让 NuRec 相机做离屏渲染，
# 显著拖低 RTF。这里在 stage 加载后显式停图并停用所有 render product；
# sim_context.step(render=True) 仍会产生雷达所需的 OnPlaybackTick。
if not _viewport:
    from pxr import UsdRender  # noqa: E402

    _stage = omni.usd.get_context().get_stage()
    _camera_graph = _stage.GetPrimAtPath("/ActionGraph_camera")
    if _camera_graph.IsValid():
        _camera_graph.GetAttribute("evaluationMode").Set("Disabled")
    _render_products = [p for p in _stage.Traverse() if p.IsA(UsdRender.Product)]
    for _rp in _render_products:
        _rp.SetActive(False)
    print(f"[headless] 相机图已禁用，停用 {len(_render_products)} 个 render product "
          "(ISAAC_VIEWPORT=0)。")

# 关键：standalone 无头里必须用 SimulationContext 来步进物理，
# 光 timeline.play()+simulation_app.update() 不会推进仿真时间(current_time 恒为 0)。
# stage_units_in_meters=1.0 与本场景 metersPerUnit=1 对齐，避免被默认 0.01(cm) 改比例。
# 这里显式设固定物理步长(physics_dt=1/PHYS_HZ)：IMU 图按物理步发布，步频必须可控且稳定；
# 下方主循环把每秒步数钉在 PHYS_HZ 上(sleep 节流)，从而 RTF≈1。
from isaacsim.core.api import SimulationContext  # noqa: E402

# 物理步频(= IMU 发布率)。场景里 IMU 图已改用 isaacsim.core.nodes.OnPhysicsStep 触发，
# /livox/imu 就按【物理步频】发布，和渲染/相机抽帧(RENDER_EVERY)完全解耦。真实 Livox
# Mid360 IMU 是 200Hz，这里默认 200；机器带不动可用 ISAAC_PHYSICS_HZ 调低(如 100)。
# rendering_dt 取和 physics_dt 相等(substeps=1) -> 每次 step() 恰好推进 1 个物理步，
# 保持主循环“一次迭代 = 一步”的假设不变；IMU 与 /clock 走物理步，
# 雷达、odom、tf、相机按 RENDER_EVERY 抽帧。
PHYS_HZ = float(_os.environ.get("ISAAC_PHYSICS_HZ", "200"))
PHYS_DT = 1.0 / PHYS_HZ
# 在 initialize_physics() 重新注册传感器【之前】设置，否则 PhysX 插件会缓存旧的
# rotationRate(场景里烘焙的 20Hz)，运行中改属性只能部分生效。记录原值仅用于打印日志；
# schema 明确定义 rotationRate=0 为“all rays at once”，即每次读取生成完整一圈。
LIDAR_PRIM = "/World/r1_pro_with_gripper/base_link/livox_frame"
_lidar_prim = omni.usd.get_context().get_stage().GetPrimAtPath(LIDAR_PRIM)
_lidar_rot_rate = float(_lidar_prim.GetAttribute("rotationRate").Get() or 20.0)
_lidar_full_scan = _os.environ.get("ISAAC_LIDAR_FULL_SCAN", "1").strip().lower() \
    not in ("0", "false", "no", "off")
if _lidar_full_scan:
    _lidar_prim.GetAttribute("rotationRate").Set(0.0)
_lidar_gate = omni.usd.get_context().get_stage().GetPrimAtPath("/ActionGraph_lidar/Gate")
if _lidar_gate.IsValid():
    # standalone 的 step(render=True) 会产生两个 playback tick；仅无头运行时设2去重。
    # USD/GUI 保持 step=1，不影响交互模式的一帧一发。
    _lidar_gate.GetAttribute("inputs:step").Set(2)
# 机器人图(/tf、/odom_gt、/joint_states)同样挂在 OnPlaybackTick 上，同样受双 tick 影响。
# 用同一套路把它的 Gate 设 2 去重：否则这些话题会成对发相同时间戳，tf2 报 TF_REPEATED_DATA。
_robot_gate = omni.usd.get_context().get_stage().GetPrimAtPath("/ActionGraph_robot/Gate")
if _robot_gate.IsValid():
    _robot_gate.GetAttribute("inputs:step").Set(2)
# /clock 与 IMU 共用物理步触发源，但通过 Gate 均匀降频。默认 200/20=每10步发布，
# 既没有 playback 双 tick 的成对突发，也不承担 200Hz ROS clock 的额外开销。
CLOCK_HZ = float(_os.environ.get("ISAAC_CLOCK_HZ", "20"))
_clock_step = max(1, round(PHYS_HZ / CLOCK_HZ)) if CLOCK_HZ > 0 else 1
_clock_gate = omni.usd.get_context().get_stage().GetPrimAtPath("/ActionGraph_imu/ClockGate")
if _clock_gate.IsValid():
    _clock_gate.GetAttribute("inputs:step").Set(_clock_step)
sim_context = SimulationContext(physics_dt=PHYS_DT, rendering_dt=PHYS_DT,
                                stage_units_in_meters=1.0)
sim_context.initialize_physics()   # 建立物理视图
sim_context.set_simulation_dt(physics_dt=PHYS_DT, rendering_dt=PHYS_DT)  # 落实物理步频
sim_context.play()                 # 开始播放；内部会先走一步把物理句柄接好
simulation_app.update()            # 让 PhysX sensor extension 完成重新注册
print(f"[headless] 物理步频 = {PHYS_HZ:.0f}Hz (ISAAC_PHYSICS_HZ) -> /livox/imu 同频发布"
      f"(OnPhysicsStep，已与渲染解耦、不再重复)。")
print(f"[headless] /clock = {PHYS_HZ / _clock_step:.0f}Hz "
      f"(ISAAC_CLOCK_HZ={CLOCK_HZ:g}，OnPhysicsStep + Gate，每 {_clock_step} 步一次)。")
print("[headless] SimulationContext.play()，目标按真实时间推进(RTF≈1，实际取决于负载)。"
      "ROS 话题应已开始发布(ros2 topic list 查看)。Ctrl+C 退出。")

report_timestep()   # 打印 dt / 步频

# ---- Headless PhysX Lidar scan adaptation ----
# 无头模式只在渲染抽帧调用 ReadLidar。rotationRate=0 让每次调用直接生成完整一圈；
# ROS 调用频率仍由 ISAAC_RENDER_HZ 控制，USD 文件本身不会被保存修改。
if _lidar_full_scan:
    print(f"[headless] 雷达适配已就绪：每个 OnPlaybackTick 生成完整一圈，"
          f"原始 rotationRate={_lidar_rot_rate:g}Hz；实际发布频率由 "
          "ISAAC_RENDER_HZ 控制 -> /livox/lidar_raw")
else:
    print(f"[headless] 雷达使用原生旋转扫描 {_lidar_rot_rate:g}Hz "
          "(ISAAC_LIDAR_FULL_SCAN=0，仅用于性能对照)。")

# 可选：降低 RTX 每像素采样数，减轻渲染(尤其 NuRec 相机场景，SPP 减半开销近乎减半)。
# ISAAC_SPP=2；0/未设 = 不改(用场景默认 8)。用 carb 运行时改，比改 usdz 里的只读元数据可靠。
_spp = int(_os.environ.get("ISAAC_SPP", "0"))
if _spp > 0:
    import carb.settings  # noqa: PLC0415
    _s = carb.settings.get_settings()
    _s.set_int("/rtx/directLighting/sampledLighting/samplesPerPixel", _spp)
    _s.set_int("/rtx/pathtracing/spp", _spp)
    _s.set_int("/rtx/pathtracing/totalSpp", _spp)
    print(f"[headless] RTX samplesPerPixel -> {_spp} (ISAAC_SPP)")

# ---- NuRec 体积裁剪(方案 A：只渲染机器人附近，给"高斯泼溅"设个渲染范围盒) ----
# 逼真环境是 NuRec 神经体(整块 ~665×675×300m)，相机每帧要泼视锥内上千万高斯 -> 很卡。
# crop 盒把远处高斯排除在渲染外，等价于"只渲染附近"。盒在体积【局部坐标】里定义，
# 这里用 USD 变换把机器人世界位置换算到体积局部系再设盒，绝不会把机器人自己裁没。
#   ISAAC_NUREC_CROP=80        盒子边长(米)，0=不裁剪(默认)。开相机做数据时设 80 试。
#   ISAAC_NUREC_CROP_FOLLOW=1  跟随机器人(动态盒)；默认 0=静态(在初始位姿处切一次)。
#   ISAAC_NUREC_CROP_EVERY=15  跟随模式下每多少帧更新一次盒(摊薄可能的加速结构重建开销)。
#   ISAAC_ROBOT_PRIM=...       机器人中心 prim，默认 base_link。
from pxr import UsdGeom, Gf  # noqa: E402

NUREC_CROP = float(_os.environ.get("ISAAC_NUREC_CROP", "0"))
NUREC_FOLLOW = _os.environ.get("ISAAC_NUREC_CROP_FOLLOW", "0") == "1"
NUREC_EVERY = max(1, int(_os.environ.get("ISAAC_NUREC_CROP_EVERY", "15")))
ROBOT_PRIM = _os.environ.get("ISAAC_ROBOT_PRIM",
                             "/World/r1_pro_with_gripper/base_link")


def _find_nurec_prim():
    stage = omni.usd.get_context().get_stage()
    for p in stage.Traverse():
        a = p.GetAttribute("omni:nurec:isNuRecVolume")
        if a and a.Get():
            return p
        if p.GetAttribute("omni:nurec:crop:minBounds").IsValid():
            return p
    return None


def _robot_world_pos():
    stage = omni.usd.get_context().get_stage()
    if NUREC_FOLLOW:
        # 跟随模式要实时物理位姿(USD 里的 xform 不随物理更新)。
        try:
            from isaacsim.core.prims import RigidPrim  # noqa: PLC0415
            pos, _ = RigidPrim(ROBOT_PRIM).get_world_poses()
            return Gf.Vec3d(float(pos[0][0]), float(pos[0][1]), float(pos[0][2]))
        except Exception:  # noqa: BLE001
            pass  # 取不到就退回 USD 静态位姿(至少不崩)
    m = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(ROBOT_PRIM))
    t = m.ExtractTranslation()
    return Gf.Vec3d(t[0], t[1], t[2])


def _apply_nurec_crop(prim, verbose=False):
    vol_w2l = UsdGeom.XformCache().GetLocalToWorldTransform(prim).GetInverse()
    c = vol_w2l.Transform(_robot_world_pos())   # 机器人中心 -> 体积局部系
    h = NUREC_CROP / 2.0
    mn = Gf.Vec3f(float(c[0] - h), float(c[1] - h), float(c[2] - h))
    mx = Gf.Vec3f(float(c[0] + h), float(c[1] + h), float(c[2] + h))
    prim.GetAttribute("omni:nurec:crop:minBounds").Set(mn)
    prim.GetAttribute("omni:nurec:crop:maxBounds").Set(mx)
    if verbose:
        print(f"[crop] 盒边长 {NUREC_CROP}m "
              f"{'跟随机器人' if NUREC_FOLLOW else '静态'} -> "
              f"local min({mn[0]:.1f},{mn[1]:.1f},{mn[2]:.1f}) "
              f"max({mx[0]:.1f},{mx[1]:.1f},{mx[2]:.1f})")


_nurec = _find_nurec_prim() if NUREC_CROP > 0 else None
if NUREC_CROP > 0 and _nurec is None:
    print("[crop] 未找到 NuRec 体积(场景可能已是网格?)，跳过裁剪。")
elif _nurec is not None:
    _apply_nurec_crop(_nurec, verbose=True)

import time  # noqa: E402
import signal  # noqa: E402
import threading  # noqa: E402

# 实时节流：把每秒步进数(=物理步频)钉在 TARGET_HZ。每次迭代恰好推进 1 个物理步，
# 循环频率就是物理步频；默认钉在 PHYS_HZ 让 RTF≈1(IMU 随之稳定在 PHYS_HZ)。
# 想改频率(或放开跑满)：设 ISAAC_HZ=200 / 100 / 0(0=不节流，能跑多快跑多快)。
TARGET_HZ = float(_os.environ.get("ISAAC_HZ", str(PHYS_HZ)))
PERIOD = (1.0 / TARGET_HZ) if TARGET_HZ > 0 else 0.0
print(f"[headless] 实时节流 TARGET_HZ={TARGET_HZ:.0f}"
      + ("(不节流)" if PERIOD == 0 else f" -> 每步目标周期 {PERIOD*1000:.2f} ms"))

# 渲染抽帧：每 RENDER_EVERY 个物理步才渲染1次。渲染帧驱动挂在 OnPlaybackTick 上的图
# (odom/tf/雷达/相机)；IMU 与 /clock 走 OnPhysicsStep。
# 默认按“目标渲染频率 ISAAC_RENDER_HZ(默认10)”自动换算，与物理步频解耦(避免物理提到
# 200Hz 后雷达/相机也飙到200Hz)；也可用 ISAAC_RENDER_EVERY 显式覆盖“每几步渲一次”。
# standalone 每个渲染步产生两个 playback tick；雷达 Gate(step=2) 去重，状态图未去重。
#   例：PHYS_HZ=200、RENDER_HZ=10 -> 状态图≈20Hz、雷达≈10Hz；
#       /clock=均匀20Hz、IMU=200Hz（后二者与渲染解耦）。
_render_every_env = _os.environ.get("ISAAC_RENDER_EVERY", "").strip()
if _render_every_env:
    RENDER_EVERY = max(1, int(_render_every_env))
else:
    _render_hz = float(_os.environ.get("ISAAC_RENDER_HZ", "10"))
    RENDER_EVERY = max(1, round(PHYS_HZ / _render_hz)) if _render_hz > 0 else 1
print(f"[headless] 渲染抽帧：每 {RENDER_EVERY} 步渲1次 "
      f"(状态图≈{2 * PHYS_HZ / RENDER_EVERY:.0f}Hz；"
      f"雷达≈{PHYS_HZ / RENDER_EVERY:.0f}Hz；"
      f"/clock={PHYS_HZ / _clock_step:.0f}Hz、IMU={PHYS_HZ:.0f}Hz 不受影响)。")

# 交互控制：无头没有 GUI 的 Stop/Play，这里给两条复位/退出入口。
# 后台线程只置标志位，真正的 reset()/退出 由主循环执行(跨线程调物理不安全)。
#   1) 终端输入 r + 回车 = 重新开始(机器人复位到 USD 初始状态)；q + 回车 = 退出。
#   2) kill -USR1 <pid> = 远程触发复位(从别的终端/机器)。
_cmd = {"reset": False, "quit": False}


def _stdin_loop():
    try:
        for line in sys.stdin:
            c = line.strip().lower()
            if c == "r":
                _cmd["reset"] = True
            elif c == "q":
                _cmd["quit"] = True
                break
    except Exception:  # noqa: BLE001
        pass  # stdin 关闭(如后台运行无终端)时静默退出线程


threading.Thread(target=_stdin_loop, daemon=True).start()
signal.signal(signal.SIGUSR1, lambda *_: _cmd.update(reset=True))
print(f"[headless] 复位/退出：终端输 r+回车=重新开始, q+回车=退出; "
      f"或远程 kill -USR1 {_os.getpid()} 复位。")

# 运行中每 5 秒打印一次实时率 RTF = 仿真时间推进 / 墙上时间。
RTF_INTERVAL = 5.0
_w_last = time.time()
_s_last = sim_context.current_time   # 当前仿真时间(秒)，由物理步进推进
_next_t = time.time()                # 下一步应开始的墙上时刻(节流用)
_frame = 0
try:
    while simulation_app.is_running():
        if _cmd["quit"]:
            print("[headless] 收到 q，退出。")
            break

        # 跟随模式：定期把裁剪盒挪到机器人当前位置(机器人走到哪只渲染哪)。
        if _nurec is not None and NUREC_FOLLOW and _frame % NUREC_EVERY == 0:
            _apply_nurec_crop(_nurec)
        _frame += 1
        if _cmd["reset"]:
            _cmd["reset"] = False
            print("[headless] 复位仿真到初始状态(机器人回到起点)…")
            sim_context.reset()          # 恢复 USD 初始位姿/关节，随后继续 play
            # 复位后重置节流与 RTF 基准，避免下一帧被当成“落后”乱追。
            _w_last = time.time()
            _s_last = sim_context.current_time
            _next_t = time.time()

        # 渲染抽帧：非渲染步只推物理(便宜)，渲染步才付相机渲染开销。
        # odom/tf/雷达/相机随渲染帧；/clock 与 IMU 由 OnPhysicsStep 驱动。
        _do_render = (_frame % RENDER_EVERY == 0)
        sim_context.step(render=_do_render)   # 步进物理 + 评估 OmniGraph(含 ROS 发布)
        # 实时节流：睡到本步的目标时刻，把频率钉在 TARGET_HZ。
        if PERIOD > 0:
            _next_t += PERIOD
            _sleep = _next_t - time.time()
            if _sleep > 0:
                time.sleep(_sleep)
            else:
                # 落后了(某帧超时)：重置基准，避免"追赶螺旋"越追越乱。
                _next_t = time.time()

        _w_now = time.time()
        if _w_now - _w_last >= RTF_INTERVAL:
            _s_now = sim_context.current_time
            ds = _s_now - _s_last
            dw = _w_now - _w_last
            rtf = ds / dw if dw > 0 else 0.0
            _target = "不节流(full speed)" if PERIOD == 0 else f"节流 {TARGET_HZ:.0f}Hz"
            print(f"[rtf] sim +{ds:.2f}s / wall +{dw:.2f}s -> RTF = {rtf:.3f}  "
                  f"(物理 {PHYS_HZ:.0f}Hz, {_target})")
            _w_last, _s_last = _w_now, _s_now
except KeyboardInterrupt:
    print("\n[headless] 收到 Ctrl+C，停止。")
finally:
    try:
        sim_context.stop()
    except Exception:  # noqa: BLE001
        pass
    simulation_app.close()
