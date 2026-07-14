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
    到 /livox/lidar_raw (frame_id=livox_frame，与真实 livox_ros_driver2 对齐)
  - 与雷达同位置一个 IMU + 发布 sensor_msgs/Imu 到 /livox/imu (frame_id=livox_frame)
  - /clock、joint_states、odom、TF 发布
  - /tf_static：base_link→livox_frame/imu、zed_link→zed_camera（自动，无需手跑 static_transform_publisher）
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
# 雷达 prim 名 = ROS frame_id。为与真实 livox_ros_driver2 驱动一致，
# 用 "livox_frame"（真实驱动点云/IMU 默认 frame 也叫 livox_frame）。
LIDAR_PRIM = BASE_LINK + "/livox_frame"
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

# 是否发布 odom->base_link 的真值 TF（PubRawTF）。
# 跑 FAST-LIO 等自带里程计/定位、由它接管 odom->base_link 时设 False：
# 否则 base_link 会有两个父帧(Isaac 与 FAST-LIO)，tf2 报 TF_MULTIPLE_AUTHORITY、位姿乱跳。
# 关掉后 /odom_gt 话题仍照常发布(PubOdom)，只是不再发这条 TF。想要 Isaac 真值 TF 时改回 True。
PUBLISH_ODOM_TF = False

CAM_W, CAM_H = 640, 360          # 降分辨率省渲染(原 1280x720，像素量降到 1/4)
CAM_HFOV_DEG = 110.0             # ZED 大致水平 FOV
ENABLE_RGB = True                # 是否发布 RGB 图
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
    # 用完整路径 + parent=None；传 parent= 会导致路径被错误拼接。
    # 与雷达同位置（LIDAR_OFFSET）安装：真实 Livox Mid-360 的 IMU 就在雷达内部，
    # 二者共用 livox_frame。这样 FAST-LIO 里 IMU<->LiDAR 外参就是单位阵。
    omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=IMU_PRIM, parent=None,
        translation=Gf.Vec3d(*LIDAR_OFFSET),
        sensor_period=-1.0,
    )
    print("[OK] IMU created:", IMU_PRIM, stage.GetPrimAtPath(IMU_PRIM).IsValid())

    # ---------- 3. Mid360 PhysX Generic Lidar ----------
    # 用 PhysX 雷达（RangeSensorCreateLidar）：对物理碰撞体做光线投射，
    # 不依赖 RTX 渲染几何，因此对高斯泼溅/NuRec 场景里有碰撞的物体也能出点。
    # 这个命令用 get_next_free_path 正确处理 parent，不会像 RTX 那样压平路径。
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

    # ---------- 3b. 关于“雷达扫到机身自身” ----------
    # 重要：PhysX Generic Lidar 用 PxScene::raycast 打【所有】碰撞体，既没有“忽略某 prim”
    # 的参数，也【不理会】UsdPhysics.FilteredPairsAPI（那个只屏蔽刚体对之间的接触求解，
    # 不作用于场景射线查询）。rangeOffset 是 RTX 雷达的参数，PhysX 雷达没有。
    # 所以自身点只能靠：①调大 min_range 做死区(会连近处真实障碍一起裁掉)，或
    # ②在下游把“机身所在区域”的点裁掉（ros2_sensors/lidar_self_filter.py，推荐，
    #   只裁机身足迹、保留其它近点）。这里不再做无效的 FilteredPairsAPI。
    print("[note] PhysX lidar 无法忽略指定 prim；机身自身点请用下游 "
          "lidar_self_filter.py 裁剪(见 README.md)，或调大 LIDAR_MIN_RANGE。")

    # ---------- 4. 机器人状态 (joint_states / odom / tf) ----------
    # PubRawTF(odom->base_link 真值 TF) 受 PUBLISH_ODOM_TF 开关控制：跑 FAST-LIO 时
    # 设为 False，让 FAST-LIO 独占 odom->base_link，避免 base_link 双父帧冲突。
    robot_nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        # Gate 去重：standalone 无头 step() 里 OnPlaybackTick 每帧评估两次，会让
        # /tf、/odom_gt、/joint_states 发出成对相同时间戳的重复消息(tf2 报 TF_REPEATED_DATA)。
        # 与雷达图同一套路：默认 step=1(GUI/交互不受影响，一帧一发)，isaac_headless.py
        # 在无头下把 /ActionGraph_robot/Gate 的 step 设为 2 去重。
        ("Gate", "isaacsim.core.nodes.IsaacSimulationGate"),
        ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PubJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("PubSensorTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("PubCamTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ]
    robot_values = [
        ("PubJoint.inputs:topicName", "/joint_states"),
        ("PubJoint.inputs:targetPrim", [Sdf.Path(BASE_LINK)]),
        ("Odom.inputs:chassisPrim", [Sdf.Path(BASE_LINK)]),
        ("PubOdom.inputs:topicName", "/odom_gt"),
        ("PubOdom.inputs:odomFrameId", "odom"),
        ("PubOdom.inputs:chassisFrameId", "base_link"),
        # PubTF 必须设 parentPrim=base_link，否则默认相对 world 发 world->base_link，
        # 会和 odom->base_link 抢 base_link 父帧。设 parentPrim 后只发
        # base_link->(arms/torso/wheels/zed 等子连杆)。
        ("PubTF.inputs:parentPrim", Sdf.Path(BASE_LINK)),
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
    ]
    robot_connects = [
        # OnPlaybackTick -> Gate -> 各发布节点(execIn)，让整组随 Gate.step 一起降频/去重。
        ("Tick.outputs:tick", "Gate.inputs:execIn"),
        ("Gate.outputs:execOut", "PubJoint.inputs:execIn"),
        ("Gate.outputs:execOut", "Odom.inputs:execIn"),
        ("Gate.outputs:execOut", "PubTF.inputs:execIn"),
        ("Gate.outputs:execOut", "PubSensorTF.inputs:execIn"),
        ("Gate.outputs:execOut", "PubCamTF.inputs:execIn"),
        ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
        ("Odom.outputs:position", "PubOdom.inputs:position"),
        ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
        ("Odom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
        ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
        ("SimTime.outputs:simulationTime", "PubJoint.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubSensorTF.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubCamTF.inputs:timeStamp"),
        ("Ctx.outputs:context", "PubJoint.inputs:context"),
        ("Ctx.outputs:context", "PubOdom.inputs:context"),
        ("Ctx.outputs:context", "PubTF.inputs:context"),
        ("Ctx.outputs:context", "PubSensorTF.inputs:context"),
        ("Ctx.outputs:context", "PubCamTF.inputs:context"),
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
        {"graph_path": "/ActionGraph_robot", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: robot_nodes,
            og.Controller.Keys.SET_VALUES: robot_values,
            og.Controller.Keys.CONNECT: robot_connects,
        },
    )
    print("[OK] robot state / clock / TF graph created (incl. /tf_static for "
          "sensors; odom->base_link TF: %s)"
          % ("ON" if PUBLISH_ODOM_TF else "OFF (FAST-LIO 接管)"))

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
    # PhysX 雷达必须由 OnPlaybackTick 读取；headless runner 会在初始化前临时设
    # rotationRate=0（schema 定义为 all rays at once），使每个渲染抽帧得到完整一圈。
    og.Controller.edit(
        {"graph_path": "/ActionGraph_lidar", "evaluator_name": "execution"},
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
                # 原始 PointCloud2（含机身自身点）。下游 lidar_self_filter.py 裁剪后
                # 发 /livox/points，再由 pc2_to_livox.py 转成 /livox/lidar (CustomMsg)。
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
    print("[OK] Mid360 point cloud graph created (OnPlaybackTick; headless full-scan adaptation)")

    # ---------- 7. IMU + 均匀 /clock 物理步图 ----------
    # IMU 每个物理步发布；/clock 也由物理步驱动，但经 Gate 每 10 步发布一次：
    # 默认物理 200Hz -> /clock 20Hz。这样避免 standalone 的双 playback tick 造成
    # /clock 成对突发，也避免直接以 200Hz 发布 ROS clock 拖慢 RTF。
    #   - 与渲染/相机抽帧解耦：headless 下即使把渲染频率降到 ~10Hz(RENDER_EVERY)，IMU
    #     仍按【物理步频】发布(真实 Mid360 IMU 是 200Hz)，不会被一起拖到 ~10Hz。
    #   - 天然去重：OnPlaybackTick 在无头 step() 里一帧会被评估两次，导致 /livox/imu
    #     每个时间戳重复发两条；OnPhysicsStep 每个物理步只触发一次，不再重复。
    # 物理步频(=IMU 发布率)在 isaac_headless.py 里用 ISAAC_PHYSICS_HZ 设置(默认 200Hz)。
    # 注意：OnPhysicsStep 只在【on-demand】图里才会触发(它靠物理回调驱动图评估，不走
    # 每帧的 simulation 管线)；若图仍是 pipelineStageSimulation，Isaac 会报
    # "Physics OnSimulationStep node detected in a non on-demand Graph" 且 IMU 不发。
    # 所以这个图必须设 pipeline_stage=ON_DEMAND。
    og.Controller.edit(
        {"graph_path": "/ActionGraph_imu", "evaluator_name": "execution",
         "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "isaacsim.core.nodes.OnPhysicsStep"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ("ClockGate", "isaacsim.core.nodes.IsaacSimulationGate"),
                ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadImu.inputs:imuPrim", [Sdf.Path(IMU_PRIM)]),
                # 与真实 livox_ros_driver2 对齐：IMU 走 /livox/imu，frame=livox_frame
                # （与雷达同帧），供 FAST-LIO 直接使用。
                ("PubImu.inputs:topicName", "/livox/imu"),
                ("PubImu.inputs:frameId", "livox_frame"),
                ("ClockGate.inputs:step", 10),
                ("PubClock.inputs:topicName", "/clock"),
            ],
            og.Controller.Keys.CONNECT: [
                # OnPhysicsStep 的执行输出口叫 outputs:step(不是 OnPlaybackTick 的 outputs:tick)
                ("Tick.outputs:step", "ReadImu.inputs:execIn"),
                ("Tick.outputs:step", "ClockGate.inputs:execIn"),
                ("ClockGate.outputs:execOut", "PubClock.inputs:execIn"),
                ("ReadImu.outputs:execOut", "PubImu.inputs:execIn"),
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
    print("[OK] physics graph created (IMU=每个物理步；/clock=物理步 Gate 后均匀 20Hz)")
    cam_topics = ""
    if ENABLE_RGB:
        cam_topics += " /zed/rgb/image_raw"
    if ENABLE_DEPTH:
        cam_topics += " /zed/depth/image_rect_raw"
    if ENABLE_RGB or ENABLE_DEPTH:
        cam_topics += " /zed/camera_info"
    print("\nDone. After pressing Play, `ros2 topic list` should show:"
          "\n  /clock /joint_states /odom_gt /tf /tf_static"
          "\n  /livox/lidar_raw /livox/imu%s"
          "\n(run ros2_sensors/bringup.launch.py -> /livox/points + /livox/lidar)"
          % cam_topics)


if __name__ == "__main__":
    setup()
