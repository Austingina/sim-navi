#!/usr/bin/env python3
"""
一键给大学城场景里的官方 Go2 加传感器 + ROS 2 发布管线（在 Isaac Sim 里跑）。

运行方式：
  ./run_isaacsim.sh --gui scene_daxuecheng_go2.usd
  （会自动 --exec 本脚本；也可 Window > Script Editor 手动 Run）

相对 r1 版差异：
  - 机器人 prim：/World/go2 ，articulation 根：/World/go2/base
  - ActionGraph 使用 *_go2 路径，避开 scene_seg_smooth 子层同名图冲突
  - IMU 用 IsaacImuSensor.Define（不依赖易未注册的 IsaacSensorCreateImuSensor 命令）
  - 默认无 ZED；Mid360 挂狗背

会发布：/clock /joint_states /odom_gt /tf /tf_static /livox/lidar_raw /livox/imu
再配 bringup_go2.launch.py → /livox/points + /livox/lidar + /gps/fix
"""
import omni.kit.app
import omni.kit.commands
import omni.usd
from pxr import Sdf, Gf, Usd, UsdGeom
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

# ================= 配置区 =================
ROBOT_PRIM = "/World/go2"
BASE_LINK = ROBOT_PRIM + "/base"
LIDAR_PRIM = BASE_LINK + "/livox_frame"
IMU_PRIM = BASE_LINK + "/imu_livox"

LIDAR_OFFSET = (0.20, 0.0, 0.28)
LIDAR_MIN_RANGE = 0.15
LIDAR_MAX_RANGE = 40.0
LIDAR_H_FOV = 360.0
LIDAR_V_FOV = 59.0
LIDAR_H_RES = 0.4
LIDAR_V_RES = 1.0
LIDAR_ROT_RATE = 20.0

PUBLISH_ODOM_TF = False

# 专用图路径：勿与 scene_seg_smooth 的 /ActionGraph_robot 等冲突
GRAPH_ROBOT = "/ActionGraph_robot_go2"
GRAPH_LIDAR = "/ActionGraph_lidar_go2"
GRAPH_IMU = "/ActionGraph_imu_go2"
# =====================================================


def _deactivate(stage, paths) -> None:
    """在 root layer 上写 active=false，并尽量让 OmniGraph 停止求值。"""
    root = stage.GetRootLayer()
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid():
            continue
        try:
            with Usd.EditContext(stage, root):
                if prim.IsActive():
                    prim.SetActive(False)
                    print("[fix] deactivated leftover", p)
        except Exception as e:
            print("[WARN] deactivate", p, e)
        # OG 可能在 open 时已缓存；再强制 disable（API 因版本而异，失败可忽略）
        try:
            g = og.get_graph_by_path(p)
            if g is not None and hasattr(g, "set_disabled"):
                g.set_disabled(True)
                print("[fix] OG disabled", p)
        except Exception:
            pass


def _remove_if_local(stage, path: str) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)


def _create_imu(stage) -> bool:
    """用 schema Define 创建 IsaacImuSensor（不依赖 kit command 注册时机）。"""
    for c in list(stage.GetPrimAtPath(BASE_LINK).GetChildren()):
        if c.GetName() in ("imu_livox", "imu_sensor"):
            stage.RemovePrim(c.GetPath())

    try:
        import omni.isaac.IsaacSensorSchema as ISS
    except ImportError as e:
        print("[ERROR] 无法 import IsaacSensorSchema:", e)
        return False

    sensor = ISS.IsaacImuSensor.Define(stage, IMU_PRIM)
    prim = sensor.GetPrim()
    if not prim.IsValid():
        print("[ERROR] IsaacImuSensor.Define 失败:", IMU_PRIM)
        return False

    try:
        ISS.IsaacBaseSensor(prim).CreateEnabledAttr(True)
    except Exception:
        if prim.HasAttribute("enabled"):
            prim.GetAttribute("enabled").Set(True)

    try:
        sensor.CreateSensorPeriodAttr().Set(-1.0)
    except Exception:
        pass

    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    for name in list(prim.GetPropertyNames()):
        if name.startswith("xformOp:"):
            prim.RemoveProperty(name)
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*LIDAR_OFFSET))
    print("[OK] IMU (IsaacImuSensor.Define):", IMU_PRIM, "type=", prim.GetTypeName())
    return True


def setup():
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.physics")
    enable_extension("isaacsim.sensors.physx")
    enable_extension("isaacsim.robot.schema")
    # 注意：禁止在此调用 app.update()。本脚本常由 --exec 异步任务里的 runpy 同步调用，
    # 再入 Kit 主循环会导致崩溃（crashreporter + tzfile assert）。

    stage = omni.usd.get_context().get_stage()
    assert stage.GetPrimAtPath(ROBOT_PRIM).IsValid(), f"找不到 {ROBOT_PRIM}"
    assert stage.GetPrimAtPath(BASE_LINK).IsValid(), f"找不到 {BASE_LINK}"

    # 关掉 r1 子层遗留图（RemovePrim 对 subLayer 图无效，会留下幽灵图导致 OG 冲突）
    _deactivate(stage, (
        "/ActionGraph_control",
        "/ActionGraph_robot",
        "/ActionGraph_lidar",
        "/ActionGraph_imu",
        "/ActionGraph_camera",
    ))
    # 删掉本脚本上次创建的 go2 专用图
    for g in (GRAPH_ROBOT, GRAPH_LIDAR, GRAPH_IMU):
        _remove_if_local(stage, g)

    print("[skip] Go2 默认无 ZED/相机图")

    # ---------- IMU ----------
    if not _create_imu(stage):
        print("[WARN] IMU 创建失败，/livox/imu 可能无数据")

    # ---------- Mid360 PhysX Lidar ----------
    for c in list(stage.GetPrimAtPath(BASE_LINK).GetChildren()):
        if c.GetName() in ("mid360", "livox_frame"):
            stage.RemovePrim(c.GetPath())
    res = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path=LIDAR_PRIM, parent=None,
        translation=Gf.Vec3d(*LIDAR_OFFSET),
        orientation=Gf.Quatd(1, 0, 0, 0),
        min_range=LIDAR_MIN_RANGE,
        max_range=LIDAR_MAX_RANGE,
        horizontal_fov=LIDAR_H_FOV,
        vertical_fov=LIDAR_V_FOV,
        horizontal_resolution=LIDAR_H_RES,
        vertical_resolution=LIDAR_V_RES,
        rotation_rate=LIDAR_ROT_RATE,
        high_lod=True,
        yaw_offset=0.0,
        enable_semantics=False,
        draw_points=False,
        draw_lines=False,
    )
    schema_obj = res[1] if isinstance(res, tuple) else res
    lidar_path = schema_obj.GetPath().pathString if schema_obj else LIDAR_PRIM
    print("[OK] Mid360 PhysX lidar:", lidar_path, stage.GetPrimAtPath(lidar_path).IsValid())
    print("[note] 机身自身点请用 bringup 里 pc2_to_livox 裁剪")

    # ---------- robot state ----------
    robot_nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        ("Gate", "isaacsim.core.nodes.IsaacSimulationGate"),
        ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PubJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("PubSensorTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ]
    robot_values = [
        ("PubJoint.inputs:topicName", "/joint_states"),
        ("PubJoint.inputs:targetPrim", [Sdf.Path(BASE_LINK)]),
        ("Odom.inputs:chassisPrim", [Sdf.Path(BASE_LINK)]),
        ("PubOdom.inputs:topicName", "/odom_gt"),
        ("PubOdom.inputs:odomFrameId", "odom"),
        ("PubOdom.inputs:chassisFrameId", "base_link"),
        ("PubTF.inputs:parentPrim", Sdf.Path(BASE_LINK)),
        ("PubTF.inputs:topicName", "/tf"),
        ("PubTF.inputs:targetPrims", [Sdf.Path(BASE_LINK)]),
        ("PubSensorTF.inputs:topicName", "/tf_static"),
        ("PubSensorTF.inputs:staticPublisher", True),
        ("PubSensorTF.inputs:parentPrim", Sdf.Path(BASE_LINK)),
        ("PubSensorTF.inputs:targetPrims",
         [Sdf.Path(lidar_path), Sdf.Path(IMU_PRIM)]),
    ]
    robot_connects = [
        ("Tick.outputs:tick", "Gate.inputs:execIn"),
        ("Gate.outputs:execOut", "PubJoint.inputs:execIn"),
        ("Gate.outputs:execOut", "Odom.inputs:execIn"),
        ("Gate.outputs:execOut", "PubTF.inputs:execIn"),
        ("Gate.outputs:execOut", "PubSensorTF.inputs:execIn"),
        ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
        ("Odom.outputs:position", "PubOdom.inputs:position"),
        ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
        ("Odom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
        ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
        ("SimTime.outputs:simulationTime", "PubJoint.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubSensorTF.inputs:timeStamp"),
        ("Ctx.outputs:context", "PubJoint.inputs:context"),
        ("Ctx.outputs:context", "PubOdom.inputs:context"),
        ("Ctx.outputs:context", "PubTF.inputs:context"),
        ("Ctx.outputs:context", "PubSensorTF.inputs:context"),
    ]
    if PUBLISH_ODOM_TF:
        robot_nodes.append(
            ("PubRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        robot_values += [
            ("PubRawTF.inputs:parentFrameId", "odom"),
            ("PubRawTF.inputs:childFrameId", "base_link"),
        ]
        robot_connects += [
            ("Odom.outputs:execOut", "PubRawTF.inputs:execIn"),
            ("Odom.outputs:position", "PubRawTF.inputs:translation"),
            ("Odom.outputs:orientation", "PubRawTF.inputs:rotation"),
            ("SimTime.outputs:simulationTime", "PubRawTF.inputs:timeStamp"),
            ("Ctx.outputs:context", "PubRawTF.inputs:context"),
        ]
    og.Controller.edit(
        {"graph_path": GRAPH_ROBOT, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: robot_nodes,
            og.Controller.Keys.SET_VALUES: robot_values,
            og.Controller.Keys.CONNECT: robot_connects,
        },
    )
    print("[OK] robot graph:", GRAPH_ROBOT)

    # ---------- lidar PC ----------
    og.Controller.edit(
        {"graph_path": GRAPH_LIDAR, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Gate", "isaacsim.core.nodes.IsaacSimulationGate"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarPointCloud"),
                ("PubPC", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadLidar.inputs:lidarPrim", [Sdf.Path(lidar_path)]),
                ("PubPC.inputs:topicName", "/livox/lidar_raw"),
                ("PubPC.inputs:frameId", "livox_frame"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "Gate.inputs:execIn"),
                ("Gate.outputs:execOut", "ReadLidar.inputs:execIn"),
                ("ReadLidar.outputs:execOut", "PubPC.inputs:execIn"),
                ("ReadLidar.outputs:data", "PubPC.inputs:data"),
                ("SimTime.outputs:simulationTime", "PubPC.inputs:timeStamp"),
                ("Ctx.outputs:context", "PubPC.inputs:context"),
            ],
        },
    )
    print("[OK] lidar graph:", GRAPH_LIDAR)

    # ---------- IMU + clock ----------
    og.Controller.edit(
        {"graph_path": GRAPH_IMU, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Gate", "isaacsim.core.nodes.IsaacSimulationGate"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Gate.inputs:step", 1),
                ("ReadImu.inputs:imuPrim", [Sdf.Path(IMU_PRIM)]),
                ("PubImu.inputs:topicName", "/livox/imu"),
                ("PubImu.inputs:frameId", "livox_frame"),
                ("PubClock.inputs:topicName", "/clock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "Gate.inputs:execIn"),
                ("Gate.outputs:execOut", "ReadImu.inputs:execIn"),
                ("Gate.outputs:execOut", "PubImu.inputs:execIn"),
                ("Gate.outputs:execOut", "PubClock.inputs:execIn"),
                ("ReadImu.outputs:linAcc", "PubImu.inputs:linearAcceleration"),
                ("ReadImu.outputs:angVel", "PubImu.inputs:angularVelocity"),
                ("ReadImu.outputs:orientation", "PubImu.inputs:orientation"),
                ("SimTime.outputs:simulationTime", "PubImu.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
                ("Ctx.outputs:context", "PubImu.inputs:context"),
                ("Ctx.outputs:context", "PubClock.inputs:context"),
            ],
        },
    )
    print("[OK] IMU+/clock graph:", GRAPH_IMU)
    print("\nDone (Go2). Play 后应有: /clock /odom_gt /livox/lidar_raw /livox/imu")
    print("再: ros2 launch ros2_sensors/bringup_go2.launch.py")


if __name__ == "__main__":
    setup()
