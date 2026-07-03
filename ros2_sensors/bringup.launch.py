#!/usr/bin/env python3
"""
一键拉起 r1_pro 仿真的 ROS 2 周边节点（Isaac 那侧的传感器/控制图请先在
Script Editor 里跑 setup_sensors.py / setup_control.py）。

本 launch 只启动**纯 ROS 节点**：
  - gps_publisher.py   (odom -> /gps/fix，georef 精确换算)
  - base_controller.py (/cmd_vel -> /joint_command，swerve 运动学，可关)
全部带 use_sim_time:=true，与 Isaac 的 /clock 对齐，避免 TF 时间外推报错。

TF 全部由 Isaac 侧 setup_sensors.py 统一发布（职责单一，不分散）：
  odom->base_link、base_link->mid360/imu/gps、zed_link->zed_camera
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

    return LaunchDescription([
        DeclareLaunchArgument("spawn_x", default_value="63.0",
                              description="机器人出生世界坐标 X (决定 GPS 原点)"),
        DeclareLaunchArgument("spawn_y", default_value="1.5",
                              description="机器人出生世界坐标 Y"),
        DeclareLaunchArgument("with_controller", default_value="true",
                              description="是否启动 swerve 底盘控制器"),

        # ---- GPS 发布 ----
        ExecuteProcess(
            cmd=["python3", os.path.join(HERE, "gps_publisher.py"),
                 "--ros-args",
                 "-p", "use_sim_time:=true",
                 "-p", ["spawn_x:=", spawn_x],
                 "-p", ["spawn_y:=", spawn_y]],
            output="screen",
        ),

        # ---- swerve 底盘控制器（可选）----
        ExecuteProcess(
            condition=IfCondition(with_controller),
            cmd=["python3", os.path.join(HERE, "base_controller.py"),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ),
    ])
