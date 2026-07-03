#!/usr/bin/env python3
"""
一键给 r1_pro 机器人加传感器 + ROS 2 发布管线（在 Isaac Sim 里跑）。

运行方式：
  打开 scene.usd（机器人已导入、已有碰撞）后，在 Isaac Sim 的
  Window > Script Editor 里粘贴本文件内容并运行；或用 standalone：
    <isaac>/python.sh setup_sensors.py

它会创建：
  - zed_link 下一个相机 (Camera prim) + 发布 rgb / camera_info（depth 可选，见 ENABLE_DEPTH）
  - base_link 上方一个 PhysX Generic Lidar (近似 Mid360) + 发布 PointCloud2
  - base_link 上一个 IMU + 发布 sensor_msgs/Imu
  - /clock、joint_states、odom、TF 发布
  - /tf_static：base_link→mid360/imu、zed_link→zed_camera（自动，无需手跑 static_transform_publisher）
全部用 OmniGraph(Action Graph) 节点，节点类型名对应 Isaac Sim 5.x。

前置：终端先 `source /opt/ros/humble/setup.bash` 再启动 Isaac Sim。
GPS 单独用 gps_publisher.py（它订阅 odom 转 NavSatFix）。
"""
import omni.kit.commands
import omni.usd
from pxr import Sdf, Gf, UsdGeom
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

# ================= 配置区（按需修改） =================
ROBOT_PRIM = "/World/r1_pro_with_gripper"   # 场景里的机器人 Xform（外层包裹）
BASE_LINK = ROBOT_PRIM + "/base_link"        # PhysX articulation 根 + IMU/Lidar 挂载
ZED_LINK = ROBOT_PRIM + "/zed_link"          # 相机挂载 link

CAMERA_PRIM = ZED_LINK + "/zed_camera"   # prim 名须与 ROS frame_id 一致
LIDAR_PRIM = BASE_LINK + "/mid360"
IMU_PRIM = BASE_LINK + "/imu"
# 注意：GPS 不在这里建 prim。gps_publisher.py 用 NavSatFix.frame_id="odom"
# 并自己发 map->odom 的 static TF（georef 旋转），无需 base_link->gps。

LIDAR_OFFSET = (0.0, 0.0, 0.5)   # Mid360 相对 base_link 的安装高度
# PhysX Generic Lidar 参数（对物理碰撞体做投射，不依赖 RTX 渲染几何）
# 近似 Livox Mid-360：360° 水平 + ~59° 垂直(-7°~+52°)
LIDAR_MIN_RANGE = 0.1
LIDAR_MAX_RANGE = 40.0
LIDAR_H_FOV = 360.0
LIDAR_V_FOV = 59.0
LIDAR_H_RES = 0.4    # 水平角分辨率(度)
LIDAR_V_RES = 1.0    # 垂直角分辨率(度) -> ~59 层
LIDAR_ROT_RATE = 20.0

CAM_W, CAM_H = 640, 360          # 降分辨率省渲染(原 1280x720，像素量降到 1/4)
CAM_HFOV_DEG = 110.0             # ZED 大致水平 FOV
ENABLE_RGB = False                # 是否发布 RGB 图
ENABLE_DEPTH = False             # 是否发布深度图(depth 是独立渲染通道，关掉省 GPU)
# 相机降频：跳过 N 帧再渲染/发布一次 -> 实际每 (N+1) 帧一次。
# 这是 Isaac 官方省 GPU 的做法(frameSkipCount 会自动设上游 IsaacSimulationGate.step=N+1，
# 降的是渲染频率不只是发布)。物理仍每 tick 步进，不受影响。
# 例：tick≈60 时 N=5 -> ~10Hz；想更省就调大。
CAM_FRAME_SKIP = 5
# 注意：RGB 和 depth 都为 False 时，整段相机图(含 render product)不创建 -> GPU 渲染负担最低
# =====================================================


def setup():
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.physics")   # IMU 创建命令 + IsaacReadIMU 节点
    enable_extension("isaacsim.sensors.physx")     # PhysX 雷达
    stage = omni.usd.get_context().get_stage()
    assert stage.GetPrimAtPath(ROBOT_PRIM).IsValid(), f"找不到 {ROBOT_PRIM}，请改 ROBOT_PRIM"

    # 重复运行时先删旧的 Action Graph，避免 CREATE_NODES 冲突
    for g in ("/ActionGraph_robot", "/ActionGraph_camera",
              "/ActionGraph_lidar", "/ActionGraph_imu"):
        if stage.GetPrimAtPath(g).IsValid():
            stage.RemovePrim(g)

    # ---------- 1. 相机 ----------
    import math
    for c in list(stage.GetPrimAtPath(ZED_LINK).GetChildren()):
        if c.GetName() in ("zed_rgbd", "zed_camera"):
            stage.RemovePrim(c.GetPath())
    cam = UsdGeom.Camera.Define(stage, Sdf.Path(CAMERA_PRIM))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))   # far 不要太大，否则视锥 gizmo 很大
    # 由水平 FOV 反推光圈/焦距
    focal = 24.0
    h_aperture = 2.0 * focal * math.tan(math.radians(CAM_HFOV_DEG) / 2.0)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(h_aperture)
    cam.CreateVerticalApertureAttr(h_aperture * CAM_H / CAM_W)

    # 相机刚性挂在 zed_link 上，朝向只相对 zed_link 给一个【常量】修正，
    # 不读世界/机器人姿态——这样换场景、机器人转向都不会再翻（和真实 ZED 一致）。
    # zed_link 在 URDF 里已是 ZED 的“光学坐标系”：+Z 向前、+X 向右、+Y 向下
    #   （zed_joint rpy≈(-110°,0,-90°)，即标准光学旋转 + 约 20° 下俯）。
    # USD 相机看自己的 -Z、+Y 朝上，与光学系正好差“绕本地 X 轴 180°”：
    #   Z_cam=-Z_zed(视线->光学+Z 前方), Y_cam=-Y_zed(像上->光学上), X_cam=+X_zed(右)。
    quatd = Gf.Quatd(0.0, 1.0, 0.0, 0.0)   # 绕本地 X 轴 180°，纯常量
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    # 清掉上一次遗留的 xformOp:* 属性，否则重复 AddOrientOp 会报错
    cam_prim = cam.GetPrim()
    for _n in list(cam_prim.GetPropertyNames()):
        if _n.startswith("xformOp:"):
            cam_prim.RemoveProperty(_n)
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quatd)   # translate=0 -> 与 zed_link 同位置
    print("[OK] camera rigidly mounted on zed_link (optical frame, const orient):", CAMERA_PRIM)

    # ---------- 2. IMU ----------
    # 用完整路径 + parent=None；传 parent= 会导致路径被错误拼接
    omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=IMU_PRIM, parent=None,
        sensor_period=-1.0,
    )
    print("[OK] IMU created:", IMU_PRIM, stage.GetPrimAtPath(IMU_PRIM).IsValid())

    # ---------- 3. Mid360 PhysX Generic Lidar ----------
    # 用 PhysX 雷达（RangeSensorCreateLidar）：对物理碰撞体做光线投射，
    # 不依赖 RTX 渲染几何，因此对高斯泼溅/NuRec 场景里有碰撞的物体也能出点。
    # 这个命令用 get_next_free_path 正确处理 parent，不会像 RTX 那样压平路径。
    for c in list(stage.GetPrimAtPath(BASE_LINK).GetChildren()):
        if c.GetName() == "mid360":
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
        high_lod=True,            # 3D 多线扫描
        yaw_offset=0.0,
        enable_semantics=False,
        draw_points=False,
        draw_lines=False,
    )
    schema_obj = res[1] if isinstance(res, tuple) else res
    lidar_path = schema_obj.GetPath().pathString if schema_obj else LIDAR_PRIM
    print("[OK] Mid360 PhysX lidar created:", lidar_path,
          stage.GetPrimAtPath(lidar_path).IsValid())

    # ---------- 4. /clock + 机器人状态 (joint_states / odom / tf) ----------
    og.Controller.edit(
        {"graph_path": "/ActionGraph_robot", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PubJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PubRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PubSensorTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PubCamTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PubClock.inputs:topicName", "/clock"),
                ("PubJoint.inputs:topicName", "/joint_states"),
                ("PubJoint.inputs:targetPrim", [Sdf.Path(BASE_LINK)]),
                ("Odom.inputs:chassisPrim", [Sdf.Path(BASE_LINK)]),
                ("PubOdom.inputs:topicName", "/odom"),
                ("PubOdom.inputs:odomFrameId", "odom"),
                ("PubOdom.inputs:chassisFrameId", "base_link"),
                ("PubRawTF.inputs:parentFrameId", "odom"),
                ("PubRawTF.inputs:childFrameId", "base_link"),
                ("PubTF.inputs:topicName", "/tf"),
                ("PubTF.inputs:targetPrims", [Sdf.Path(BASE_LINK)]),
                ("PubSensorTF.inputs:topicName", "/tf_static"),
                ("PubSensorTF.inputs:staticPublisher", True),
                ("PubSensorTF.inputs:parentPrim", Sdf.Path(BASE_LINK)),
                ("PubSensorTF.inputs:targetPrims",
                 [Sdf.Path(lidar_path), Sdf.Path(IMU_PRIM)]),
                ("PubCamTF.inputs:topicName", "/tf_static"),
                ("PubCamTF.inputs:staticPublisher", True),
                ("PubCamTF.inputs:parentPrim", Sdf.Path(ZED_LINK)),
                ("PubCamTF.inputs:targetPrims", [Sdf.Path(CAMERA_PRIM)]),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "PubClock.inputs:execIn"),
                ("Tick.outputs:tick", "PubJoint.inputs:execIn"),
                ("Tick.outputs:tick", "Odom.inputs:execIn"),
                ("Tick.outputs:tick", "PubTF.inputs:execIn"),
                ("Tick.outputs:tick", "PubSensorTF.inputs:execIn"),
                ("Tick.outputs:tick", "PubCamTF.inputs:execIn"),
                ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
                ("Odom.outputs:execOut", "PubRawTF.inputs:execIn"),
                ("Odom.outputs:position", "PubOdom.inputs:position"),
                ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
                ("Odom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
                ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
                ("Odom.outputs:position", "PubRawTF.inputs:translation"),
                ("Odom.outputs:orientation", "PubRawTF.inputs:rotation"),
                ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubJoint.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubRawTF.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubSensorTF.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubCamTF.inputs:timeStamp"),
                ("Ctx.outputs:context", "PubClock.inputs:context"),
                ("Ctx.outputs:context", "PubJoint.inputs:context"),
                ("Ctx.outputs:context", "PubOdom.inputs:context"),
                ("Ctx.outputs:context", "PubRawTF.inputs:context"),
                ("Ctx.outputs:context", "PubTF.inputs:context"),
                ("Ctx.outputs:context", "PubSensorTF.inputs:context"),
                ("Ctx.outputs:context", "PubCamTF.inputs:context"),
            ],
        },
    )
    print("[OK] robot state / clock / TF graph created (incl. /tf_static for sensors)")

    # ---------- 5. 相机图 (RGB / depth / camera_info 各自可开关) ----------
    if not (ENABLE_RGB or ENABLE_DEPTH):
        print("[skip] camera graph disabled (ENABLE_RGB=False, ENABLE_DEPTH=False) "
              "-> no render product, lowest GPU render load")
    else:
        # camera_info 跟随渲染产品发布；RGB/depth 各自按开关追加
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
            {"graph_path": "/ActionGraph_camera", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: cam_nodes,
                og.Controller.Keys.SET_VALUES: cam_values,
                og.Controller.Keys.CONNECT: cam_connect,
            },
        )
        print("[OK] camera graph created (rgb=%s depth=%s frameSkip=%d -> 1/%d ticks)"
              % (ENABLE_RGB, ENABLE_DEPTH, CAM_FRAME_SKIP, CAM_FRAME_SKIP + 1))

    # ---------- 6. Mid360 点云图 (PhysX) ----------
    # PhysX 雷达：IsaacReadLidarPointCloud 读取一整圈点云 -> ROS2PublishPointCloud
    og.Controller.edit(
        {"graph_path": "/ActionGraph_lidar", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarPointCloud"),
                ("PubPC", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadLidar.inputs:lidarPrim", [Sdf.Path(lidar_path)]),
                ("PubPC.inputs:topicName", "/mid360/points"),
                ("PubPC.inputs:frameId", "mid360"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "ReadLidar.inputs:execIn"),
                ("ReadLidar.outputs:execOut", "PubPC.inputs:execIn"),
                ("ReadLidar.outputs:data", "PubPC.inputs:data"),
                ("SimTime.outputs:simulationTime", "PubPC.inputs:timeStamp"),
                ("Ctx.outputs:context", "PubPC.inputs:context"),
            ],
        },
    )
    print("[OK] Mid360 point cloud graph created (PhysX)")

    # ---------- 7. IMU 图 ----------
    og.Controller.edit(
        {"graph_path": "/ActionGraph_imu", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadImu.inputs:imuPrim", [Sdf.Path(IMU_PRIM)]),
                ("PubImu.inputs:topicName", "/imu"),
                ("PubImu.inputs:frameId", "imu"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "ReadImu.inputs:execIn"),
                ("ReadImu.outputs:execOut", "PubImu.inputs:execIn"),
                ("ReadImu.outputs:linAcc", "PubImu.inputs:linearAcceleration"),
                ("ReadImu.outputs:angVel", "PubImu.inputs:angularVelocity"),
                ("ReadImu.outputs:orientation", "PubImu.inputs:orientation"),
                ("SimTime.outputs:simulationTime", "PubImu.inputs:timeStamp"),
                ("Ctx.outputs:context", "PubImu.inputs:context"),
            ],
        },
    )
    print("[OK] IMU graph created")
    cam_topics = ""
    if ENABLE_RGB:
        cam_topics += " /zed/rgb/image_raw"
    if ENABLE_DEPTH:
        cam_topics += " /zed/depth/image_rect_raw"
    if ENABLE_RGB or ENABLE_DEPTH:
        cam_topics += " /zed/camera_info"
    print("\nDone. After pressing Play, `ros2 topic list` should show:"
          "\n  /clock /joint_states /odom /tf /tf_static"
          "\n  /mid360/points /imu%s" % cam_topics)


if __name__ == "__main__":
    setup()
