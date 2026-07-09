#!/usr/bin/env python3
"""
PointCloud2 -> livox_ros_driver2/CustomMsg 转换节点。

背景：Isaac 仿真的 PhysX 雷达只能发 sensor_msgs/PointCloud2，而基于 Livox 的
SLAM（FAST-LIO2 / Point-LIO 等）默认订阅 livox_ros_driver2/msg/CustomMsg 的
/livox/lidar。本节点把（自身点裁剪后的）PointCloud2 转成 CustomMsg，让仿真与真机
用完全一样的 topic + 消息类型，SLAM 侧无需改代码。

管线：
  Isaac  --/livox/lidar_raw(PointCloud2)-->  lidar_self_filter.py
         --/livox/points(PointCloud2)-->     pc2_to_livox.py(本节点)
         --/livox/lidar(CustomMsg)-->         FAST-LIO / SLAM

字段映射：
  x,y,z          <- PointCloud2 的 x/y/z (float32)
  reflectivity   <- intensity 字段（若无则 0），裁到 0~255
  tag / line     <- 0（仿真无 tag / 无逐线 ring 信息）
  offset_time    <- 按点序在一帧扫描周期(point_time_span)内线性铺开（用于去畸变的近似）

前置：运行前必须 source 编译好 livox_ros_driver2 的工作区 install，
否则 import livox_ros_driver2.msg 会失败：
  source install/setup.bash

运行：
  python3 pc2_to_livox.py --ros-args -p use_sim_time:=true
可调参数：
  -p input_topic:=/livox/points -p output_topic:=/livox/lidar
  -p frame_id:=livox_frame -p point_time_span:=0.05
  -p downsample_stride:=3   # 每 N 点留 1；瓶颈是逐点重建 CustomMsg，降采样直接提速。1=不降
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

from livox_ros_driver2.msg import CustomMsg, CustomPoint


class Pc2ToLivox(Node):
    def __init__(self):
        super().__init__("pc2_to_livox")
        self.declare_parameter("input_topic", "/livox/points")
        self.declare_parameter("output_topic", "/livox/lidar")
        # 空字符串 = 沿用输入点云的 frame_id；否则强制覆盖成该值。
        self.declare_parameter("frame_id", "livox_frame")
        # 一帧扫描的时间跨度(秒)，offset_time 按点序在此区间线性铺开。
        # Mid360 rotationRate=20 -> 每圈 0.05s。
        self.declare_parameter("point_time_span", 0.05)
        # 降采样步长：每 stride 个点留 1 个(x[::stride])。逐点重建 CustomMsg 是本节点的
        # 瓶颈(Python 循环 + rosidl 序列化 2 万+ 嵌套消息)，砍点数直接线性提速。
        # 3 -> 2 万点降到 ~7 千，足够 FAST-LIO(它内部还会体素降采样)且能稳到 20Hz。
        # 设 1 = 不降采样。
        self.declare_parameter("downsample_stride", 3)

        self.frame_id = self.get_parameter("frame_id").value
        self.span_ns = int(self.get_parameter("point_time_span").value * 1e9)
        self.stride = max(1, int(self.get_parameter("downsample_stride").value))
        self.in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # 订阅 Best Effort（能收 Reliable 或 Best Effort 的上游）；
        # 发布 Reliable（`ros2 topic echo` 默认 reliable，SLAM 也多用 reliable）。
        sub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        pub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(CustomMsg, out_topic, pub_qos)
        self.sub = self.create_subscription(
            PointCloud2, self.in_topic, self.on_pc, sub_qos)

        self.n_recv = 0
        self.last_out = 0
        self._warned = False
        self.create_timer(3.0, self._status)
        self.get_logger().info(
            f"pc2->CustomMsg: {self.in_topic}(BestEffort) -> {out_topic}(Reliable)"
            f"  frame_id={self.frame_id or '(keep input)'}"
            f"  span={self.span_ns/1e6:.1f}ms"
            f"  downsample_stride={self.stride}"
            + ("(不降采样)" if self.stride == 1 else f"(每{self.stride}点留1)"))

    def _status(self):
        if self.n_recv == 0:
            self.get_logger().warn(
                f"还没收到 {self.in_topic}：确认 lidar_self_filter.py 在发 "
                f"/livox/points（或把 input_topic 指到 /livox/lidar_raw）。"
                f"`ros2 topic hz {self.in_topic}` 查一下。")
        else:
            self.get_logger().info(
                f"ok: 已转换 {self.n_recv} 帧，最近 {self.last_out} 点 -> CustomMsg")

    def _field_offset(self, msg, name):
        for f in msg.fields:
            if f.name == name:
                return f.offset
        return None

    def on_pc(self, msg: PointCloud2):
        self.n_recv += 1
        try:
            n = msg.width * msg.height
            xo = self._field_offset(msg, "x")
            yo = self._field_offset(msg, "y")
            zo = self._field_offset(msg, "z")
            io = self._field_offset(msg, "intensity")
            if n == 0 or xo is None or yo is None or zo is None:
                if not self._warned:
                    self.get_logger().warn("点云为空或缺 x/y/z，发空 CustomMsg")
                    self._warned = True
                self.pub.publish(self._make_msg(msg, np.empty((0, 3), np.float32),
                                                None, 0))
                return

            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

            def f32(off):
                return raw[:, off:off + 4].copy().view(np.float32).reshape(-1)

            x, y, z = f32(xo), f32(yo), f32(zo)
            xyz = np.stack((x, y, z), axis=1)
            inten = f32(io) if io is not None else None

            finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            xyz = xyz[finite]
            if inten is not None:
                inten = inten[finite]

            # 降采样：进逐点循环前砍点数(瓶颈在这个循环)。切片是 O(1) 视图，几乎零开销。
            if self.stride > 1:
                xyz = xyz[::self.stride]
                if inten is not None:
                    inten = inten[::self.stride]

            out = self._make_msg(msg, xyz, inten, xyz.shape[0])
            self.last_out = xyz.shape[0]
            self.pub.publish(out)
        except Exception as e:
            if not self._warned:
                self.get_logger().error(f"转换异常({e})，跳过该帧")
                self._warned = True

    def _make_msg(self, src: PointCloud2, xyz: np.ndarray, inten, npts: int):
        msg = CustomMsg()
        msg.header = src.header
        if self.frame_id:
            msg.header.frame_id = self.frame_id
        # timebase = 帧时间戳(ns)，offset_time 相对它
        msg.timebase = (int(src.header.stamp.sec) * 1_000_000_000
                        + int(src.header.stamp.nanosec))
        msg.point_num = npts
        msg.lidar_id = 0
        # rsvd (uint8[3]) 默认即全 0，不手动赋值以免 rosidl 固定数组类型检查报错

        if npts == 0:
            msg.points = []
            return msg

        # offset_time 线性铺开；reflectivity 从 intensity 映射(0~255)
        if npts > 1:
            offs = (np.arange(npts, dtype=np.float64) / (npts - 1)
                    * self.span_ns).astype(np.uint32)
        else:
            offs = np.zeros(1, dtype=np.uint32)
        if inten is not None:
            refl = np.clip(inten, 0, 255).astype(np.uint8)
        else:
            refl = np.zeros(npts, dtype=np.uint8)

        xf = xyz[:, 0].astype(np.float32)
        yf = xyz[:, 1].astype(np.float32)
        zf = xyz[:, 2].astype(np.float32)

        points = []
        for i in range(npts):
            p = CustomPoint()
            p.offset_time = int(offs[i])
            p.x = float(xf[i])
            p.y = float(yf[i])
            p.z = float(zf[i])
            p.reflectivity = int(refl[i])
            p.tag = 0
            p.line = 0
            points.append(p)
        msg.points = points
        return msg


def main():
    rclpy.init()
    node = Pc2ToLivox()
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
