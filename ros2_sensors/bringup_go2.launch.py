#!/usr/bin/env python3
"""大学城 Go2 bringup：对齐 outdoor 栈，**默认关闭** swerve 底盘控制器。

与 bringup.launch.py 相同的 GPS / livox 流水线，差异：
  - with_controller 默认 false（Go2 由 unitree_rl_lab 策略消费 /cmd_vel，不是 base_controller）
  - 默认 georef_daxuecheng.json
  - 可选启动 go2_topic_bridge（odom 命名空间重映射）

用法（仓库根目录）：
  source setup_ros_local.sh
  source ~/Desktop/fast_lio2/ros2_ws/install/setup.bash   # livox_ros_driver2
  # 终端1: ./run_isaacsim.sh --gui scene_daxuecheng_go2.usd
  #         Script Editor 运行 ros2_sensors/setup_sensors_go2.py，再点 Play
  ros2 launch ros2_sensors/bringup_go2.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))
# Humble 绑定系统 Python 3.10；勿用 conda 的 python3
PYTHON3 = "/usr/bin/python3"


def generate_launch_description():
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    with_controller = LaunchConfiguration("with_controller")
    with_livox_pipeline = LaunchConfiguration("with_livox_pipeline")
    with_points = LaunchConfiguration("with_points")
    with_gps_map_tf = LaunchConfiguration("with_gps_map_tf")
    with_odom_tf = LaunchConfiguration("with_odom_tf")
    with_go2_bridge = LaunchConfiguration("with_go2_bridge")
    with_camera_tf = LaunchConfiguration("with_camera_tf")
    georef_json = LaunchConfiguration("georef_json")

    default_georef = os.path.normpath(
        os.path.join(HERE, "..", "scene_tools", "georef_daxuecheng.json"))

    return LaunchDescription([
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION",
            os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "7")),
        SetEnvironmentVariable(
            "ROS_LOCALHOST_ONLY", os.environ.get("ROS_LOCALHOST_ONLY", "1")),

        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument(
            "with_controller", default_value="false",
            description="Go2 默认 false：勿启动 r1 swerve base_controller"),
        DeclareLaunchArgument("with_livox_pipeline", default_value="true"),
        DeclareLaunchArgument("with_points", default_value="true"),
        DeclareLaunchArgument("with_gps_map_tf", default_value="false"),
        DeclareLaunchArgument("with_odom_tf", default_value="false"),
        DeclareLaunchArgument(
            "with_go2_bridge", default_value="false",
            description="是否启动 go2_topic_bridge（仅当 odom 在 /unitree_go2/* 命名空间时）"),
        DeclareLaunchArgument(
            "with_camera_tf", default_value="true",
            description="发布 base_link→zed_link→zed_camera 静态 TF（对齐 setup_sensors_go2 挂载）"),
        DeclareLaunchArgument("georef_json", default_value=default_georef),

        # 安装点与 base 同姿态；zed_camera 为光学系（+Z 前、+X 右、+Y 下），对齐正立画面
        ExecuteProcess(
            condition=IfCondition(with_camera_tf),
            cmd=[
                "ros2", "run", "tf2_ros", "static_transform_publisher",
                "--x", "0.32", "--y", "0", "--z", "0.12",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "base_link", "--child-frame-id", "zed_link",
            ],
            output="log",
        ),
        ExecuteProcess(
            condition=IfCondition(with_camera_tf),
            cmd=[
                "ros2", "run", "tf2_ros", "static_transform_publisher",
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
                "--frame-id", "zed_link", "--child-frame-id", "zed_camera",
            ],
            output="log",
        ),
        ExecuteProcess(
            condition=IfCondition(with_odom_tf),
            cmd=[PYTHON3, os.path.join(HERE, "odom_tf_publisher.py"),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[PYTHON3, os.path.join(HERE, "gps_publisher.py"),
                 "--ros-args",
                 "-p", "use_sim_time:=true",
                 "-p", ["georef_json:=", georef_json],
                 "-p", ["spawn_x:=", spawn_x],
                 "-p", ["spawn_y:=", spawn_y],
                 "-p", ["publish_map_odom_tf:=", with_gps_map_tf]],
            output="screen",
        ),
        ExecuteProcess(
            condition=IfCondition(with_controller),
            cmd=[PYTHON3, os.path.join(HERE, "base_controller.py"),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ),
        ExecuteProcess(
            condition=IfCondition(with_livox_pipeline),
            cmd=[PYTHON3, os.path.join(HERE, "pc2_to_livox.py"),
                 "--ros-args", "-p", "use_sim_time:=true",
                 "-p", ["publish_points:=", with_points]],
            output="log",
        ),
        ExecuteProcess(
            condition=IfCondition(with_go2_bridge),
            cmd=[PYTHON3, os.path.join(HERE, "go2_topic_bridge.py"),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ),
    ])
