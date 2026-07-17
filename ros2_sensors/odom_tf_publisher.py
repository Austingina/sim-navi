#!/usr/bin/env python3
"""把 /odom_gt (nav_msgs/Odometry, Isaac 真值里程计) 广播成 odom->base_link 的 TF。

背景：本项目为了不和 FAST-LIO 抢 base_link 父帧，把 Isaac 图里发真值 TF 的 PubRawTF
移除了(setup_sensors.py 的 PUBLISH_ODOM_TF=False)。但纯可视化 / 不跑 FAST-LIO 时，
rviz 的 Fixed Frame=odom 需要 odom->base_link 才能把机器人摆在里程计原点上，否则
RobotModel 连不到 odom、显示不出来。

这个可开关的小节点直接消费已有的 /odom_gt，把它重播成 TF —— 不需要改任何 USD。

⚠️ 跑 FAST-LIO(或任何自带 odom->base_link 的定位)时不要开它，否则 base_link 双父帧、
tf2 报 TF_MULTIPLE_AUTHORITY、位姿乱跳。用 bringup 的 with_odom_tf:=false 关掉(默认就是关)。

用法：
    python3 odom_tf_publisher.py --ros-args -p use_sim_time:=true \
        -p odom_topic:=/odom_gt
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__("odom_tf_publisher")
        self._odom_topic = self.declare_parameter(
            "odom_topic", "/odom_gt").get_parameter_value().string_value
        self._br = TransformBroadcaster(self)
        # 队列给大点，避免高频里程计时丢帧
        self.create_subscription(Odometry, self._odom_topic, self._cb, 50)
        self.get_logger().info(
            f"广播 odom->base_link TF (来源 {self._odom_topic})。"
            "跑 FAST-LIO 时请关掉本节点(with_odom_tf:=false)，避免抢 base_link 父帧。")

    def _cb(self, msg: Odometry):
        t = TransformStamped()
        # 直接沿用消息里的 frame(odom)/child(base_link)/时间戳，与 /odom_gt 完全一致
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self._br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
