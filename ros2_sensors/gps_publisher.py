#!/usr/bin/env python3
"""
GPS (NavSatFix) 发布节点 —— Isaac Sim 无原生 GPS，这里用机器人里程计真值位姿
换算成经纬度。订阅 /odom，发布 /gps/fix。

地理配准全自动、无需手工标定：重建工具(L2Pro)在 point_cloud.ply 头里写好了
    offset(局部原点的真实 UTM 坐标)  epsg 32649(UTM 49N)  scale 1  shift 0
即「局部坐标 = 真实 UTM 坐标 - offset，尺度=1，轴向与 UTM 对齐」。
这些由 scene_tools/make_georef.py 落成 georef.json，本节点启动时自动加载。

换算：
    UTM = origin + scale * R(yaw) * odom位移         # origin = offset + spawn
    lat, lon = UTM_to_WGS84(UTM, zone, north)        # 标准闭式公式，无需 pyproj
本场景 scale=1、yaw=0；通常你只需设 spawn_x/spawn_y(机器人出生世界坐标)。

另发静态 TF map -> odom（ENU 对齐）供 rviz_satellite 定向瓦片。
NavSatFix.frame_id 用 base_link（GPS 传感器所在帧）：rviz_satellite 用它锚定瓦片，
填错(如 odom)会让机器人比底图超前整段 odom 位移。

运行：
  source /opt/ros/jazzy/setup.bash && python3 gps_publisher.py
"""
import json
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import StaticTransformBroadcaster

# georef.json 默认位置（由 scene_tools/make_georef.py 从 PLY 生成）
_DEFAULT_GEOREF = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "scene_tools", "georef.json"))


def utm_to_latlon(easting, northing, zone=49, north=True):
    """WGS84 UTM -> (lat, lon)，单位度。标准 USGS 反算级数公式。"""
    a = 6378137.0
    f = 1.0 / 298.257223563
    k0 = 0.9996
    e0 = 500000.0
    n0 = 0.0 if north else 10000000.0
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    x = easting - e0
    y = northing - n0
    m = y / k0
    mu = m / (a * (1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256))
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    phi1 = (mu
            + (3 * e1 / 2 - 27 * e1 ** 3 / 32) * math.sin(2 * mu)
            + (21 * e1 ** 2 / 16 - 55 * e1 ** 4 / 32) * math.sin(4 * mu)
            + (151 * e1 ** 3 / 96) * math.sin(6 * mu)
            + (1097 * e1 ** 4 / 512) * math.sin(8 * mu))
    c1 = ep2 * math.cos(phi1) ** 2
    t1 = math.tan(phi1) ** 2
    n1 = a / math.sqrt(1 - e2 * math.sin(phi1) ** 2)
    r1 = a * (1 - e2) / (1 - e2 * math.sin(phi1) ** 2) ** 1.5
    d = x / (n1 * k0)
    lat = phi1 - (n1 * math.tan(phi1) / r1) * (
        d ** 2 / 2
        - (5 + 3 * t1 + 10 * c1 - 4 * c1 ** 2 - 9 * ep2) * d ** 4 / 24
        + (61 + 90 * t1 + 298 * c1 + 45 * t1 ** 2 - 252 * ep2 - 3 * c1 ** 2) * d ** 6 / 720)
    lon0 = math.radians(zone * 6 - 183)
    lon = lon0 + (
        d
        - (1 + 2 * t1 + c1) * d ** 3 / 6
        + (5 - 2 * c1 + 28 * t1 - 3 * c1 ** 2 + 8 * ep2 + 24 * t1 ** 2) * d ** 5 / 120) / math.cos(phi1)
    return math.degrees(lat), math.degrees(lon)


class GpsPublisher(Node):
    def __init__(self):
        super().__init__("gps_publisher")
        # 地理配准默认从 georef.json 自动加载(覆盖 utm_zone/north/offset_*/scale)；
        # 下面这些 declare 仅作 georef.json 缺失时的回退默认值。
        self.declare_parameter("georef_json", _DEFAULT_GEOREF)
        self.declare_parameter("utm_zone", 49)
        self.declare_parameter("utm_north", True)
        self.declare_parameter("offset_x", 801814.1094338207)   # PLY offsetx
        self.declare_parameter("offset_y", 2499371.9000139288)  # PLY offsety
        self.declare_parameter("offset_z", 29.3283)             # PLY offsetz
        self.declare_parameter("scale", 1.0)                    # 1米odom对应多少米UTM(本场景=1)
        self.declare_parameter("yaw_deg", 0.0)                  # odom 系 -> ENU 旋转(度, CCW正)
        # 机器人出生世界坐标：决定 GPS 原点平移。换出生点时改这里。
        self.declare_parameter("spawn_x", 5.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.0)
        # 通用
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("fix_topic", "/gps/fix")
        self.declare_parameter("frame_id", "base_link")        # GPS 传感器所在帧(勿填 odom)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("noise_std_m", 0.0)

        self.yaw = math.radians(self.get_parameter("yaw_deg").value)
        self.scale = self.get_parameter("scale").value
        self.zone = self.get_parameter("utm_zone").value
        self.north = self.get_parameter("utm_north").value
        self.ox = self.get_parameter("offset_x").value
        self.oy = self.get_parameter("offset_y").value
        self.oz = self.get_parameter("offset_z").value
        self._load_georef(self.get_parameter("georef_json").value)
        self.spawn_x = self.get_parameter("spawn_x").value
        self.spawn_y = self.get_parameter("spawn_y").value
        self.spawn_z = self.get_parameter("spawn_z").value
        # GPS 原点 = offset + scale*R(yaw)*spawn。
        # spawn 与 odom 同在“新场景世界系”，换算到已配准系时同样要过 yaw/scale，
        # 否则 yaw≠0 的场景(如新区域相对旧场景转了 ~91°)原点会算偏几米。
        # 旧场景 yaw=0、scale=1 时退化为 origin = offset + spawn，行为不变。
        ct0, st0 = math.cos(self.yaw), math.sin(self.yaw)
        self.origin_e = self.ox + self.scale * (ct0 * self.spawn_x - st0 * self.spawn_y)
        self.origin_n = self.oy + self.scale * (st0 * self.spawn_x + ct0 * self.spawn_y)
        self.oz = self.oz + self.spawn_z
        self.frame_id = self.get_parameter("frame_id").value
        self.map_frame = self.get_parameter("map_frame").value
        self.noise = self.get_parameter("noise_std_m").value

        self.pub = self.create_publisher(
            NavSatFix, self.get_parameter("fix_topic").value, 10)
        self.sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)
        self.tf_static = StaticTransformBroadcaster(self)
        self._publish_map_to_odom_tf()

        la, lo = utm_to_latlon(self.origin_e, self.origin_n, self.zone, self.north)
        self.get_logger().info(
            f"GPS zone={self.zone} scale={self.scale} "
            f"yaw={math.degrees(self.yaw):.4f}deg "
            f"spawn=({self.spawn_x},{self.spawn_y},{self.spawn_z}) "
            f"origin lat/lon=({la:.7f},{lo:.7f}) frame_id={self.frame_id} -> "
            f"{self.get_parameter('fix_topic').value}")

    def _load_georef(self, path):
        """加载 georef.json，用 PLY 既定的 zone/offset/scale 覆盖参数(权威单一源)。"""
        if not path or not os.path.isfile(path):
            self.get_logger().warn(
                f"georef.json 未找到({path})，沿用参数默认值。可运行 "
                f"scene_tools/make_georef.py 生成。")
            return
        try:
            with open(path) as f:
                g = json.load(f)
        except Exception as e:
            self.get_logger().error(f"georef.json 解析失败: {e}，沿用参数默认值。")
            return
        self.zone = int(g.get("utm_zone", self.zone))
        self.north = bool(g.get("utm_north", self.north))
        self.ox = float(g.get("offset_x", self.ox))
        self.oy = float(g.get("offset_y", self.oy))
        self.oz = float(g.get("offset_z", self.oz))
        # 仅当用户没有显式覆盖 scale 时采用 georef 里的 scale(本场景=1)
        if self.get_parameter("scale").value == 1.0:
            self.scale = float(g.get("scale", self.scale))
        # yaw_deg 也可写进 georef(如新区域配准出的 ~91°)；用户没显式传 yaw_deg 时采用它
        if "yaw_deg" in g and self.get_parameter("yaw_deg").value == 0.0:
            self.yaw = math.radians(float(g["yaw_deg"]))
        self.get_logger().info(
            f"已加载 georef.json: EPSG=326{self.zone} "
            f"offset=({self.ox},{self.oy},{self.oz}) scale={self.scale} "
            f"yaw={math.degrees(self.yaw):.4f}deg source={g.get('source','?')}")

    def _publish_map_to_odom_tf(self):
        """ENU map 帧 -> odom（yaw_deg 旋转），供 rviz_satellite 定向卫星瓦片。"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = "odom"
        half = self.yaw / 2.0
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self.tf_static.sendTransform(t)
        self.get_logger().info(
            f"static TF {self.map_frame} -> odom "
            f"(yaw={math.degrees(self.yaw):.4f}deg) for rviz_satellite")

    def on_odom(self, msg: Odometry):
        east = msg.pose.pose.position.x
        north = msg.pose.pose.position.y
        up = msg.pose.pose.position.z
        if self.noise > 0.0:
            import random
            east += random.gauss(0.0, self.noise)
            north += random.gauss(0.0, self.noise)

        # 相似变换：UTM = origin + scale * R(yaw) * odom位移
        ct, st = math.cos(self.yaw), math.sin(self.yaw)
        e = self.scale * (ct * east - st * north) + self.origin_e
        n = self.scale * (st * east + ct * north) + self.origin_n
        lat, lon = utm_to_latlon(e, n, self.zone, self.north)
        alt = up + self.oz

        fix = NavSatFix()
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = self.frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt
        var = (self.noise ** 2) if self.noise > 0 else 1.0
        fix.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var * 4]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(fix)


def main():
    rclpy.init()
    node = GpsPublisher()
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
