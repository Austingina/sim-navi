#!/usr/bin/env python3
"""Go2 仿真话题 → 大学城 outdoor 栈话题重映射。

官方 unitree_rl_lab **不**发布 /livox/*；传感器优先用
`setup_sensors_go2.py` + `bringup_go2.launch.py`（Isaac 图直接发
/livox/lidar_raw、/livox/imu、/odom_gt）。

本节点用于：
  1) 社区 Go2 ROS 桥 / 其它命名空间的 odom → `/odom_gt`
  2) 可选把非标准 cmd_vel 转发到 `/cmd_vel`（Lab 策略订阅）
  3) 可选 PointCloud2 透传（真正 CustomMsg 仍走 pc2_to_livox）

推荐控制链路：
  Nav2 → /cmd_vel_nav → smoother → CM → /cmd_vel →（Lab play_cmd_vel / 策略）

用法:
  source setup_ros_local.sh
  python3 ros2_sensors/go2_topic_bridge.py --ros-args \\
    -p use_sim_time:=true \\
    -p odom_in:=/unitree_go2/odom \\
    -p odom_out:=/odom_gt
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2


class Go2TopicBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_topic_bridge")
        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("odom_in", "/unitree_go2/odom")
        self.declare_parameter("odom_out", "/odom_gt")
        self.declare_parameter("cmd_vel_in", "")
        self.declare_parameter("cmd_vel_out", "/cmd_vel")
        self.declare_parameter("cloud_in", "")
        self.declare_parameter("cloud_out", "/livox/lidar_raw")

        odom_in = self.get_parameter("odom_in").value
        odom_out = self.get_parameter("odom_out").value
        self._odom_pub = self.create_publisher(Odometry, odom_out, 10)
        self.create_subscription(Odometry, odom_in, self._on_odom, 10)
        self._odom_count = 0

        cmd_in = str(self.get_parameter("cmd_vel_in").value or "")
        cmd_out = self.get_parameter("cmd_vel_out").value
        if cmd_in:
            self._cmd_pub = self.create_publisher(Twist, cmd_out, 10)
            self.create_subscription(Twist, cmd_in, self._on_cmd, 10)
            self.get_logger().info(f"cmd_vel bridge {cmd_in} → {cmd_out}")
        else:
            self._cmd_pub = None

        cloud_in = str(self.get_parameter("cloud_in").value or "")
        cloud_out = self.get_parameter("cloud_out").value
        if cloud_in:
            self._cloud_pub = self.create_publisher(PointCloud2, cloud_out, 10)
            self.create_subscription(PointCloud2, cloud_in, self._on_cloud, 10)
            self.get_logger().info(f"PointCloud2 bridge {cloud_in} → {cloud_out}")
        else:
            self._cloud_pub = None

        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info(
            f"Go2 bridge: odom {odom_in} → {odom_out}. "
            "官方路径请用 setup_sensors_go2 + bringup_go2；"
            "CustomMsg /livox/lidar 由 pc2_to_livox 生成。"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_pub.publish(msg)
        self._odom_count += 1

    def _on_cmd(self, msg: Twist) -> None:
        if self._cmd_pub is not None:
            self._cmd_pub.publish(msg)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._cloud_pub is not None:
            self._cloud_pub.publish(msg)

    def _heartbeat(self) -> None:
        self.get_logger().info(f"odom forwarded msgs={self._odom_count}")


def main() -> None:
    rclpy.init()
    node = Go2TopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
