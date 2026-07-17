#!/usr/bin/env python3
"""
一键拉起 r1_pro 仿真的 ROS 2 周边节点（Isaac 那侧的传感器/控制图请先在
Script Editor 里跑 setup_sensors.py / setup_control.py）。

本 launch 只启动**纯 ROS 节点**：
  - gps_publisher.py     (odom -> /gps/fix，georef 精确换算；默认不发 map->odom TF)
  - base_controller.py   (/cmd_vel -> /joint_command，swerve 运动学，可关)
  - lidar_self_filter.py (裁掉机身自身点：/livox/lidar_raw -> /livox/points，可关)
  - pc2_to_livox.py      (/livox/points(PointCloud2) -> /livox/lidar(CustomMsg)，可关)
全部带 use_sim_time:=true，与 Isaac 的 /clock 对齐，避免 TF 时间外推报错。

雷达 topic 与真实 livox_ros_driver2 对齐（frame_id=livox_frame）：
  /livox/lidar_raw (PointCloud2,Isaac原始) -> /livox/points (PointCloud2,已裁机身)
  -> /livox/lidar (livox_ros_driver2/CustomMsg，给 FAST-LIO 等 SLAM)
注意：pc2_to_livox.py 需要 import livox_ros_driver2.msg，运行前先
  source install/setup.bash（编译过 livox_ros_driver2 的工作区）。

TF 全部由 Isaac 侧 setup_sensors.py 统一发布（职责单一，不分散）：
  odom->base_link、base_link->livox_frame/imu/gps、zed_link->zed_camera
（gps 是 setup_sensors 里建的零偏移占位 prim）。所以这里**不再起任何
static_transform_publisher**。

用法（在仓库根目录下）：
  source /opt/ros/humble/setup.bash
  ros2 launch ros2_sensors/bringup.launch.py

地理配准(UTM zone/offset/scale)由 georef.json 自动加载（scene_tools/make_georef.py
从 PLY 生成），**无需手工标定**。你通常只需设机器人出生世界坐标：
  ros2 launch .../bringup.launch.py spawn_x:=30.0 spawn_y:=-12.0
关掉底盘控制器：
  ros2 launch .../bringup.launch.py with_controller:=false
GPS 的 map->odom 静态 TF 默认已关（跑 FAST-LIO 时避免抢 odom 父帧）；只有想给
rviz_satellite 卫星图定向时再打开：
  ros2 launch .../bringup.launch.py with_gps_map_tf:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    with_controller = LaunchConfiguration("with_controller")
    with_self_filter = LaunchConfiguration("with_self_filter")
    with_livox_custommsg = LaunchConfiguration("with_livox_custommsg")
    with_gps_map_tf = LaunchConfiguration("with_gps_map_tf")
    with_odom_tf = LaunchConfiguration("with_odom_tf")
    georef_json = LaunchConfiguration("georef_json")

    default_georef = os.path.normpath(
        os.path.join(HERE, "..", "scene_tools", "georef.json"))

    return LaunchDescription([
        DeclareLaunchArgument("spawn_x", default_value="0.0",
                              description="机器人出生世界坐标 X (决定 GPS 原点)"),
        DeclareLaunchArgument("spawn_y", default_value="0.0",
                              description="机器人出生世界坐标 Y"),
        DeclareLaunchArgument("with_controller", default_value="true",
                              description="是否启动 swerve 底盘控制器"),
        DeclareLaunchArgument("with_self_filter", default_value="true",
                              description="是否裁掉雷达扫到的机身自身点"
                                          "(/livox/lidar_raw -> /livox/points)"),
        DeclareLaunchArgument("with_livox_custommsg", default_value="true",
                              description="是否把 /livox/points 转成 "
                                          "/livox/lidar(livox_ros_driver2/CustomMsg)；"
                                          "需先 source 编译过 livox_ros_driver2 的 install"),
        DeclareLaunchArgument("with_gps_map_tf", default_value="false",
                              description="GPS 是否发 map->odom 静态 TF；默认关"
                                          "(跑 FAST-LIO 等自带 odom/map 帧时避免抢父帧)。"
                                          "只想要卫星图定向时设 true"),
        DeclareLaunchArgument("georef_json", default_value=default_georef,
                              description="地理配准文件；换场景传对应的(如 "
                                          "scene_tools/georef_square.json)"),
        DeclareLaunchArgument("with_odom_tf", default_value="false",
                              description="是否把 /odom_gt 真值里程计广播成 odom->base_link "
                                          "TF(纯可视化/无 FAST-LIO 时用；rviz 摆机器人需要它)。"
                                          "默认关：跑 FAST-LIO 时开会抢 base_link 父帧"),

        # ---- odom->base_link TF（可选，默认关）----
        # 把已有的 /odom_gt(真值 Odometry) 重播成 TF；不改任何 USD。
        # 跑 FAST-LIO 时保持关闭，避免和它抢 base_link 父帧(TF_MULTIPLE_AUTHORITY)。
        ExecuteProcess(
            condition=IfCondition(with_odom_tf),
            cmd=["python3", os.path.join(HERE, "odom_tf_publisher.py"),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ),

        # ---- GPS 发布 ----
        ExecuteProcess(
            cmd=["python3", os.path.join(HERE, "gps_publisher.py"),
                 "--ros-args",
                 "-p", "use_sim_time:=true",
                 "-p", ["georef_json:=", georef_json],
                 "-p", ["spawn_x:=", spawn_x],
                 "-p", ["spawn_y:=", spawn_y],
                 "-p", ["publish_map_odom_tf:=", with_gps_map_tf]],
            output="screen",
        ),

        # ---- swerve 底盘控制器（可选）----
        ExecuteProcess(
            condition=IfCondition(with_controller),
            cmd=["python3", os.path.join(HERE, "base_controller.py"),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ),

        # ---- 雷达自身点裁剪（可选）：/livox/lidar_raw -> /livox/points ----
        # output="log"：日志只进 ~/.ros/log，不刷屏(这俩节点高频、输出很吵)。
        ExecuteProcess(
            condition=IfCondition(with_self_filter),
            cmd=["python3", os.path.join(HERE, "lidar_self_filter.py"),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="log",
        ),

        # # ---- PointCloud2 -> CustomMsg（可选）：/livox/points -> /livox/lidar ----
        # ExecuteProcess(
        #     condition=IfCondition(with_livox_custommsg),
        #     cmd=["python3", os.path.join(HERE, "pc2_to_livox.py"),
        #          "--ros-args", "-p", "use_sim_time:=true"],
        #     output="log",
        # ),
    ])
