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
  - Mid360 挂狗背；**默认挂前置相机**（话题与 r1 对齐：/zed/rgb/image_raw，便于 outdoor.rviz）

会发布：/clock /joint_states /odom_gt /tf /tf_static /livox/lidar_raw /livox/imu
       /zed/rgb/image_raw /zed/camera_info（需渲染：GUI 或 ISAAC_VIEWPORT=1）
再配 bringup_go2.launch.py → /livox/points + /livox/lidar + /gps/fix
"""
import math
import omni.kit.app
import omni.kit.commands
import omni.usd
from pxr import Sdf, Gf, Usd, UsdGeom, UsdLux
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

# ================= 配置区 =================
ROBOT_PRIM = "/World/go2"
BASE_LINK = ROBOT_PRIM + "/base"
LIDAR_PRIM = BASE_LINK + "/livox_frame"
IMU_PRIM = BASE_LINK + "/imu_livox"
# 官方 Go2 无 zed_link：自建光学挂载点，话题仍用 /zed/* 以兼容 outdoor.rviz
ZED_LINK = BASE_LINK + "/zed_link"
CAMERA_PRIM = ZED_LINK + "/zed_camera"
CAM_OFFSET = (0.32, 0.0, 0.12)  # 大致狗头前方
CAM_W, CAM_H = 640, 360
CAM_HFOV_DEG = 90.0
ENABLE_RGB = True
ENABLE_DEPTH = False
CAM_FRAME_SKIP = 5  # tick 抽帧；实际约 1/(skip+1)

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
GRAPH_CAMERA = "/ActionGraph_camera_go2"
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


def _clear_xform_ops(prim) -> None:
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    for name in list(prim.GetPropertyNames()):
        if name.startswith("xformOp:"):
            prim.RemoveProperty(name)


def _create_camera(stage) -> bool:
    """前置相机：视线沿 base +X，画面上方向 base +Z。

    USD 相机看本地 -Z、+Y 为像上，且变换是行向量（p' = p * M）。
    直接写已核对的四元数，避免欧拉乘序把视线转到 +Y、画面横滚 90°。
    """
    for name in ("zed_link", "zed_camera", "zed_rgbd", "front_camera"):
        p = stage.GetPrimAtPath(BASE_LINK + "/" + name)
        if p.IsValid():
            stage.RemovePrim(p.GetPath())

    # zed_link 只做安装点（与 base 同姿态），光学系由 bringup 静态 TF 发布
    zed = UsdGeom.Xform.Define(stage, Sdf.Path(ZED_LINK))
    _clear_xform_ops(zed.GetPrim())
    UsdGeom.Xformable(zed).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*CAM_OFFSET)
    )

    cam = UsdGeom.Camera.Define(stage, Sdf.Path(CAMERA_PRIM))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
    focal = 24.0
    h_aperture = 2.0 * focal * math.tan(math.radians(CAM_HFOV_DEG) / 2.0)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(h_aperture)
    cam.CreateVerticalApertureAttr(h_aperture * CAM_H / CAM_W)

    # 行向量下：local X=-Y_base（右），Y=+Z_base（上），Z=-X_base（视线 -Z = +X 前）
    _clear_xform_ops(cam.GetPrim())
    UsdGeom.Xformable(cam).AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(0.5, 0.5, -0.5, -0.5)
    )
    print("[OK] camera forward+upright:", CAMERA_PRIM, "offset=", CAM_OFFSET)
    return True


def _create_sky(stage) -> None:
    """穹顶光当天空。大学城只有地面/建筑，射线打空就是黑，不是缺一块网格。"""
    path = "/World/sky"
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid() and prim.GetTypeName() != "DomeLight":
        stage.RemovePrim(path)
        prim = None
    light = UsdLux.DomeLight.Define(stage, path)
    light.CreateIntensityAttr(800.0)
    light.CreateColorAttr(Gf.Vec3f(0.45, 0.65, 1.0))
    try:
        import carb
        carb.settings.get_settings().set("/rtx/background/source/type", "domeLight")
    except Exception as e:
        print("[WARN] 未能把 RTX 背景切到 domeLight:", e)
    print("[OK] sky dome:", path)


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
    for g in (GRAPH_ROBOT, GRAPH_LIDAR, GRAPH_IMU, GRAPH_CAMERA):
        _remove_if_local(stage, g)

    # ---------- 相机 ----------
    cam_ok = False
    if ENABLE_RGB or ENABLE_DEPTH:
        cam_ok = _create_camera(stage)
        _create_sky(stage)
    else:
        print("[skip] Go2 相机关闭（ENABLE_RGB=ENABLE_DEPTH=False）")

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
    sensor_targets = [Sdf.Path(lidar_path), Sdf.Path(IMU_PRIM)]
    # 注意：zed_link 是普通 Xform，PoseTree 会报 eInvalid；相机 TF 用 RawTransform 手发
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
        ("PubSensorTF.inputs:targetPrims", sensor_targets),
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
    if cam_ok:
        # zed_link 是普通 Xform，PoseTree 会 eInvalid；静态 TF 改由 bringup_go2
        # （tf2_ros static_transform_publisher，QoS 正确）发布 base_link→zed_link→zed_camera
        print("[note] 相机 TF 请用 bringup_go2（static_transform_publisher），"
              "勿依赖 PoseTree")
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

    # ---------- 相机 RGB / camera_info ----------
    if not cam_ok:
        print("[skip] camera graph（无相机 prim）")
    else:
        cam_nodes = [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("RP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ]
        cam_values = [
            ("RP.inputs:cameraPrim", [Sdf.Path(CAMERA_PRIM)]),
            ("RP.inputs:width", CAM_W),
            ("RP.inputs:height", CAM_H),
            ("Info.inputs:topicName", "/zed/camera_info"),
            ("Info.inputs:frameId", "zed_camera"),
            ("Info.inputs:frameSkipCount", CAM_FRAME_SKIP),
        ]
        cam_connect = [
            ("Tick.outputs:tick", "RP.inputs:execIn"),
            ("RP.outputs:execOut", "Info.inputs:execIn"),
            ("RP.outputs:renderProductPath", "Info.inputs:renderProductPath"),
            ("Ctx.outputs:context", "Info.inputs:context"),
        ]
        if ENABLE_RGB:
            cam_nodes.append(("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"))
            cam_values += [
                ("Rgb.inputs:type", "rgb"),
                ("Rgb.inputs:topicName", "/zed/rgb/image_raw"),
                ("Rgb.inputs:frameId", "zed_camera"),
                ("Rgb.inputs:frameSkipCount", CAM_FRAME_SKIP),
            ]
            cam_connect += [
                ("RP.outputs:execOut", "Rgb.inputs:execIn"),
                ("RP.outputs:renderProductPath", "Rgb.inputs:renderProductPath"),
                ("Ctx.outputs:context", "Rgb.inputs:context"),
            ]
        if ENABLE_DEPTH:
            cam_nodes.append(("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"))
            cam_values += [
                ("Depth.inputs:type", "depth"),
                ("Depth.inputs:topicName", "/zed/depth/image_rect_raw"),
                ("Depth.inputs:frameId", "zed_camera"),
                ("Depth.inputs:frameSkipCount", CAM_FRAME_SKIP),
            ]
            cam_connect += [
                ("RP.outputs:execOut", "Depth.inputs:execIn"),
                ("RP.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
                ("Ctx.outputs:context", "Depth.inputs:context"),
            ]
        og.Controller.edit(
            {"graph_path": GRAPH_CAMERA, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: cam_nodes,
                og.Controller.Keys.SET_VALUES: cam_values,
                og.Controller.Keys.CONNECT: cam_connect,
            },
        )
        print(
            "[OK] camera graph:", GRAPH_CAMERA,
            "(rgb=%s depth=%s frameSkip=%d -> 1/%d ticks)"
            % (ENABLE_RGB, ENABLE_DEPTH, CAM_FRAME_SKIP, CAM_FRAME_SKIP + 1),
        )

    cam_topics = ""
    if cam_ok and ENABLE_RGB:
        cam_topics += " /zed/rgb/image_raw"
    if cam_ok and ENABLE_DEPTH:
        cam_topics += " /zed/depth/image_rect_raw"
    if cam_ok and (ENABLE_RGB or ENABLE_DEPTH):
        cam_topics += " /zed/camera_info"
    print("\nDone (Go2). Play 后应有: /clock /odom_gt /livox/lidar_raw /livox/imu" + cam_topics)
    print("相机需渲染：GUI 或 headless 设 ISAAC_VIEWPORT=1")
    print("再: ros2 launch ros2_sensors/bringup_go2.launch.py")


if __name__ == "__main__":
    setup()
