#!/usr/bin/env python3
"""
雷达单节点流水线：Isaac PointCloud2 -> (裁机身 + 降采样) -> livox_ros_driver2/CustomMsg。

背景：Isaac 仿真的 PhysX 雷达只能发 sensor_msgs/PointCloud2，而基于 Livox 的
SLAM（FAST-LIO2 / Point-LIO 等）默认订阅 livox_ros_driver2/msg/CustomMsg 的
/livox/lidar。同时 Isaac 雷达会扫到机器人自己的机身（PxScene::raycast 打所有碰撞体，
没有“忽略某 prim”功能），需要按机身包围盒裁掉自身点。

本节点把原来的两个节点【合并成一个】：
  旧: Isaac --/livox/lidar_raw--> lidar_self_filter --/livox/points--> pc2_to_livox --/livox/lidar-->
  新: Isaac --/livox/lidar_raw--> 本节点(裁机身+降采样) --> /livox/points(RViz) + /livox/lidar(SLAM)
合并省掉了中间一整跳 DDS + 一次 PointCloud2 反序列化，降低端到端延迟与抖动。

裁机身：在 base_link 系里 |x|<=half_x 且 |y|<=half_y 且 z_min<=z<=z_max 的点 = 机身 -> 删。
坐标换算：雷达 livox_frame 刚性挂 base_link、无旋转、仅上移 sensor_z，故
base_link 系 z = livox_frame z + sensor_z，无需查 TF。输出点仍用 livox_frame 坐标。

字段映射（CustomMsg）：
  x/y/z          <- PointCloud2 的 x/y/z（livox_frame）
  reflectivity   <- intensity 裁到 0~255
  offset_time    <- 按点序在一帧扫描周期(point_time_span)内线性铺开（去畸变近似）

前置：运行前必须 source 编译好 livox_ros_driver2 的工作区 install，
否则 import livox_ros_driver2.msg 会失败：
  source install/setup.bash

用法：
  python3 pc2_to_livox.py --ros-args -p use_sim_time:=true
可调参数：
  -p input_topic:=/livox/lidar_raw -p output_topic:=/livox/lidar
  -p points_topic:=/livox/points -p publish_points:=true   # 关掉可省一次 tobytes+发布
  -p frame_id:=livox_frame -p point_time_span:=0.1
  -p downsample_stride:=3   # 每 N 点留 1；仅作用于 CustomMsg，/livox/points 保持全分辨率
  # 机身包围盒(base_link 系，米)：
  -p half_x:=0.45 -p half_y:=0.40 -p z_min:=-0.30 -p z_max:=1.80 -p sensor_z:=0.5
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
        self.declare_parameter("input_topic", "/livox/lidar_raw")
        self.declare_parameter("output_topic", "/livox/lidar")
        self.declare_parameter("points_topic", "/livox/points")
        # 是否发滤波后的 PointCloud2(给 RViz)。纯 SLAM 无头跑可设 false，省一次 tobytes+发布。
        self.declare_parameter("publish_points", True)
        # 空字符串 = 沿用输入点云的 frame_id；否则强制覆盖成该值。
        self.declare_parameter("frame_id", "livox_frame")
        # 一帧扫描的时间跨度(秒)，offset_time 按点序在此区间线性铺开。
        # 实际帧率(见 `ros2 topic hz /livox/lidar`)约 10Hz -> 每帧约 0.1s。
        self.declare_parameter("point_time_span", 0.1)
        # 降采样步长：每 stride 个点留 1 个(x[::stride])。逐点重建 CustomMsg 是本节点的
        # 瓶颈(Python 循环 + rosidl 序列化 2 万+ 嵌套消息)，砍点数直接线性提速。设 1 = 不降。
        self.declare_parameter("downsample_stride", 3)
        # 机身包围盒(base_link 系，米)。默认值偏保守，请对着 RViz 收紧。
        self.declare_parameter("half_x", 0.45)     # 机身半长(前后)
        self.declare_parameter("half_y", 0.40)     # 机身半宽(左右)
        self.declare_parameter("z_min", -0.30)     # 机身底(base_link 系)
        self.declare_parameter("z_max", 1.80)      # 机身顶(含立柱/机械臂根部)
        self.declare_parameter("sensor_z", 0.5)    # livox_frame 相对 base_link 的安装高度

        self.frame_id = self.get_parameter("frame_id").value
        self.span_ns = int(self.get_parameter("point_time_span").value * 1e9)
        self.stride = max(1, int(self.get_parameter("downsample_stride").value))
        self.hx = self.get_parameter("half_x").value
        self.hy = self.get_parameter("half_y").value
        self.zmin = self.get_parameter("z_min").value
        self.zmax = self.get_parameter("z_max").value
        self.sz = self.get_parameter("sensor_z").value
        self.in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        pts_topic = self.get_parameter("points_topic").value
        want_points = bool(self.get_parameter("publish_points").value)

        # 订阅 Best Effort（能收 Reliable 或 Best Effort 的上游）；
        # 发布 Reliable（`ros2 topic echo` 默认 reliable，SLAM 也多用 reliable）。
        sub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        pub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(CustomMsg, out_topic, pub_qos)
        self.pub_pts = (self.create_publisher(PointCloud2, pts_topic, pub_qos)
                        if want_points else None)
        self.sub = self.create_subscription(
            PointCloud2, self.in_topic, self.on_pc, sub_qos)

        self.n_recv = 0
        self.last_in = 0
        self.last_kept = 0
        self.last_out = 0
        self._warned = False
        self._empty_warned = False
        self.create_timer(3.0, self._status)
        self.get_logger().info(
            f"livox 流水线: {self.in_topic}(BestEffort) -> "
            f"{pts_topic}{'(全分辨率)' if want_points else '(关闭)'} + "
            f"{out_topic}(CustomMsg,每{self.stride}点留1)  "
            f"box(base_link) |x|<={self.hx} |y|<={self.hy} z∈[{self.zmin},{self.zmax}] "
            f"sensor_z={self.sz}  span={self.span_ns/1e6:.1f}ms "
            f"frame_id={self.frame_id or '(keep input)'}")

    def _status(self):
        if self.n_recv == 0:
            self.get_logger().warn(
                f"还没收到 {self.in_topic}：确认 Isaac 已 Play、该话题在发、"
                f"QoS 兼容 Best Effort。`ros2 topic hz {self.in_topic}` 查一下。")
        else:
            self.get_logger().info(
                f"ok: 收到 {self.n_recv} 帧，最近 in={self.last_in} "
                f"-> 裁机身后 {self.last_kept} -> CustomMsg {self.last_out} 点")

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
            xo = self._field_offset(msg, "x")
            yo = self._field_offset(msg, "y")
            zo = self._field_offset(msg, "z")
            io = self._field_offset(msg, "intensity")
            if n == 0 or xo is None or yo is None or zo is None:
                if n != 0 and not self._warned:
                    self.get_logger().warn("点云缺 x/y/z：/livox/points 原样转发、"
                                           "CustomMsg 发空。")
                    self._warned = True
                self.last_kept = self.last_out = 0
                if self.pub_pts is not None:
                    self.pub_pts.publish(msg)
                self.pub.publish(self._make_custom(msg, np.empty((0, 3), np.float32),
                                                   None, 0))
                return

            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

            def f32(off):
                return raw[:, off:off + 4].copy().view(np.float32).reshape(-1)

            x = f32(xo)
            y = f32(yo)
            z_raw = f32(zo)                    # livox_frame z（输出坐标用它，勿原地改）
            zc = z_raw + self.sz              # -> base_link 系 z，仅用于机身盒判定

            # 机身盒内 = 自身点 -> 删；再与有限性合并。
            inside = ((np.abs(x) <= self.hx) & (np.abs(y) <= self.hy) &
                      (zc >= self.zmin) & (zc <= self.zmax))
            keep = ((~inside) & np.isfinite(x) & np.isfinite(y)
                    & np.isfinite(z_raw))
            self.last_kept = int(keep.sum())

            if self.last_kept == 0 and not self._empty_warned:
                self.get_logger().warn(
                    "裁机身后 0 点：包围盒可能太大把整片点云都当成机身，"
                    "调小 half_x/half_y 或收紧 z_min/z_max。这一帧原样兜底转发。")
                self._empty_warned = True
                if self.pub_pts is not None:
                    self.pub_pts.publish(msg)
                self.pub.publish(self._make_custom(msg, np.empty((0, 3), np.float32),
                                                   None, 0))
                return

            # /livox/points：滤波后【全分辨率】(RViz 用)，直接掩码原始字节，保留所有字段。
            if self.pub_pts is not None:
                self.pub_pts.publish(self._make_points(msg, raw[keep]))

            # /livox/lidar：滤波 + 降采样后的 CustomMsg(给 FAST-LIO)。
            xyz = np.stack((x[keep], y[keep], z_raw[keep]), axis=1)
            inten = f32(io)[keep] if io is not None else None
            if self.stride > 1:
                xyz = xyz[::self.stride]
                if inten is not None:
                    inten = inten[::self.stride]
            self.last_out = xyz.shape[0]
            self.pub.publish(self._make_custom(msg, xyz, inten, self.last_out))
        except Exception as e:  # noqa: BLE001
            if not self._warned:
                self.get_logger().error(f"处理异常({e})，跳过该帧")
                self._warned = True

    def _make_points(self, src: PointCloud2, kept_raw: np.ndarray):
        """把掩码后的原始点字节重新打包成 PointCloud2(结构、字段、字节序全不变)。"""
        out = PointCloud2()
        out.header = src.header
        if self.frame_id:
            out.header.frame_id = self.frame_id
        out.height = 1
        out.width = int(kept_raw.shape[0])
        out.fields = src.fields
        out.is_bigendian = src.is_bigendian
        out.point_step = src.point_step
        out.row_step = src.point_step * out.width
        out.is_dense = True
        out.data = kept_raw.tobytes()
        return out

    def _make_custom(self, src: PointCloud2, xyz: np.ndarray, inten, npts: int):
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

        # 向量化构建 points：先把各列 numpy 数组一次性 .tolist() 转成原生 Python 序列
        # (避免逐点 numpy 标量索引 + float()/int() 装箱，这才是原 for 循环的大头)，
        # 再用列表推导 + zip 一次建完，每点只调一次 CustomPoint(**kwargs)。
        # tag/line 默认即 0，不显式赋值(省两次 setattr)；嵌套消息的 C 序列化无法避开，
        # 但 Python 侧开销已压到最低。
        offs_l = offs.tolist()
        refl_l = refl.tolist()
        xf_l = xyz[:, 0].astype(np.float32).tolist()
        yf_l = xyz[:, 1].astype(np.float32).tolist()
        zf_l = xyz[:, 2].astype(np.float32).tolist()

        msg.points = [
            CustomPoint(offset_time=o, x=x, y=y, z=z, reflectivity=r)
            for o, x, y, z, r in zip(offs_l, xf_l, yf_l, zf_l, refl_l)
        ]
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
