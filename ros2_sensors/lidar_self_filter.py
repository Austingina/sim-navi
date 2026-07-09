#!/usr/bin/env python3
"""
Mid360 自身点裁剪 —— 把落在“机器人机身足迹”内的点去掉后重新发布。

背景：Isaac 的 PhysX Generic Lidar 用 PxScene::raycast 打【所有】碰撞体，没有“忽略某
prim”的功能，也不理会 UsdPhysics.FilteredPairsAPI。于是雷达会扫到机器人自己的机身。
真实机器人（含 Livox）都是在下游按机身几何把这些点滤掉（如 ROS 的 robot_body_filter）。
本节点做同样的事，但用一个简单的“机身包围盒”（在 base_link 系里）即可，够用且好调：
    在 base_link 系里，|x|<=half_x 且 |y|<=half_y 且 z_min<=z<=z_max 的点 = 机身 -> 删除。
盒子外的点（含近处真实障碍）全部保留 —— 比单纯调大 min_range（各方向一刀切）干净。

坐标换算：雷达 livox_frame 刚性挂在 base_link 上、无旋转、仅上移 sensor_z（=setup_sensors.py 的
LIDAR_OFFSET z）。所以 base_link 坐标 = livox_frame 坐标 +(0,0,sensor_z)，无需查 TF。

按 PointCloud2 原始字节做掩码，保留 intensity 等所有字段、不改点结构。

运行：
  source /opt/ros/jazzy/setup.bash
  python3 lidar_self_filter.py --ros-args -p use_sim_time:=true
调盒子（RViz 里对着看，逐步收紧到刚好盖住机身）：
  ... -p half_x:=0.45 -p half_y:=0.40 -p z_min:=-0.30 -p z_max:=1.80 -p sensor_z:=0.5
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class LidarSelfFilter(Node):
    def __init__(self):
        super().__init__("lidar_self_filter")
        self.declare_parameter("input_topic", "/livox/lidar_raw")
        self.declare_parameter("output_topic", "/livox/points")
        # 机身包围盒（base_link 系，米）。默认值偏保守，请对着 RViz 收紧。
        self.declare_parameter("half_x", 0.45)     # 机身半长(前后)
        self.declare_parameter("half_y", 0.40)     # 机身半宽(左右)
        self.declare_parameter("z_min", -0.30)     # 机身底(base_link 系)
        self.declare_parameter("z_max", 1.80)      # 机身顶(含立柱/机械臂根部)
        self.declare_parameter("sensor_z", 0.5)    # livox_frame 相对 base_link 的安装高度

        self.hx = self.get_parameter("half_x").value
        self.hy = self.get_parameter("half_y").value
        self.zmin = self.get_parameter("z_min").value
        self.zmax = self.get_parameter("z_max").value
        self.sz = self.get_parameter("sensor_z").value

        # 订阅用 Best Effort（最宽松，能收 Isaac 无论 reliable/best-effort 的点云）；
        # 发布用 Reliable —— 这样 `ros2 topic echo`(默认 reliable) 和 RViz(best effort)
        # 都能读到。之前发布端用 best-effort，echo 默认 reliable 会“收不到->看着像没点”。
        sub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        pub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self.sub = self.create_subscription(
            PointCloud2, self.in_topic, self.on_pc, sub_qos)
        self._warned = False
        self._err_warned = False
        self._empty_warned = False
        self.n_recv = 0
        self.last_in = 0
        self.last_out = 0
        self.create_timer(3.0, self._status)   # 定期自诊断
        self.get_logger().info(
            f"self-filter: box(base_link) |x|<={self.hx} |y|<={self.hy} "
            f"z∈[{self.zmin},{self.zmax}] sensor_z={self.sz}  "
            f"{self.in_topic}(BestEffort) -> {out_topic}(Reliable)")

    def _status(self):
        if self.n_recv == 0:
            self.get_logger().warn(
                f"还没收到 {self.in_topic} 的点云：确认 Isaac 已 Play、该话题在发、"
                f"且发布端 QoS 兼容 Best Effort。`ros2 topic hz {self.in_topic}` 查一下。")
        else:
            self.get_logger().info(
                f"ok: 收到 {self.n_recv} 帧，最近 in={self.last_in} -> out={self.last_out} 点")

    def _field_offset(self, msg, name):
        for f in msg.fields:
            if f.name == name:
                return f.offset
        return None

    def on_pc(self, msg: PointCloud2):
        self.n_recv += 1
        try:
            n = msg.width * msg.height
            self.last_in = n
            if n == 0:
                self.last_out = 0
                self.pub.publish(msg)
                return
            xo = self._field_offset(msg, "x")
            yo = self._field_offset(msg, "y")
            zo = self._field_offset(msg, "z")
            if xo is None or yo is None or zo is None:
                if not self._warned:
                    self.get_logger().warn("点云缺少 x/y/z 字段，原样转发")
                    self._warned = True
                self.last_out = n
                self.pub.publish(msg)
                return

            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

            def f32(off):
                return raw[:, off:off + 4].copy().view(np.float32).reshape(-1)

            x = f32(xo)
            y = f32(yo)
            z = f32(zo) + self.sz          # -> base_link 系 z

            inside = ((np.abs(x) <= self.hx) & (np.abs(y) <= self.hy) &
                      (z >= self.zmin) & (z <= self.zmax))
            keep = (~inside) & np.isfinite(x) & np.isfinite(y) & np.isfinite(f32(zo))

            kept = raw[keep]
            self.last_out = int(kept.shape[0])
            if self.last_out == 0 and not self._empty_warned:
                self.get_logger().warn(
                    "过滤后 0 点：包围盒可能太大把整片点云都当成机身了，"
                    "调小 half_x/half_y 或收紧 z_min/z_max。这一帧先原样转发兜底。")
                self._empty_warned = True
                self.pub.publish(msg)   # 兜底：宁可带自身点，也不发空
                return

            out = PointCloud2()
            out.header = msg.header
            out.height = 1
            out.width = self.last_out
            out.fields = msg.fields
            out.is_bigendian = msg.is_bigendian
            out.point_step = msg.point_step
            out.row_step = msg.point_step * out.width
            out.is_dense = True
            out.data = kept.tobytes()
            self.pub.publish(out)
        except Exception as e:
            # 兜底：任何异常都原样转发，绝不让下游拿到空话题
            if not self._err_warned:
                self.get_logger().error(f"过滤异常({e})，改为原样转发原始点云")
                self._err_warned = True
            self.last_out = self.last_in
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = LidarSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
