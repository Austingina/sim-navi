#!/usr/bin/env python3
"""阶段 B：量化 /livox 点云叠影风险（系统 /usr/bin/python3 + Humble）。

用法（Isaac scene_daxuecheng_go2 + bringup_go2 已发 /livox/points 或 /livox/lidar_raw）:

  source setup_ros_local.sh
  python3 scripts/lidar_ghost_check.py --topic /livox/points --seconds 20

指标：
  - frame_to_frame_jaccard：相邻帧体素占用交并比（越低越像叠影/拖影）
  - z_std_near：近处点云 z 标准差（平地被扫成「台阶」时升高）
静止基线应明显高于行走；若行走 jaccard 相对静止掉很多 → 感知误判风险高。
"""
from __future__ import annotations

import argparse
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def _voxels(pts: np.ndarray, res: float) -> set[tuple[int, int, int]]:
    if pts.size == 0:
        return set()
    q = np.floor(pts / res).astype(np.int32)
    return {tuple(x) for x in q}


class GhostCheck(Node):
    def __init__(self, topic: str, voxel: float, near_r: float) -> None:
        super().__init__("lidar_ghost_check")
        self.voxel = voxel
        self.near_r = near_r
        self.prev: set[tuple[int, int, int]] | None = None
        self.jaccards: deque[float] = deque(maxlen=500)
        self.z_stds: deque[float] = deque(maxlen=500)
        self.n = 0
        self.create_subscription(PointCloud2, topic, self._on_cloud, 10)
        self.get_logger().info(f"订阅 {topic} voxel={voxel} near_r={near_r}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        raw = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not raw:
            return
        arr = np.asarray(raw)
        if arr.dtype.names:
            pts = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
        else:
            pts = np.asarray(arr, dtype=np.float64)
            if pts.ndim != 2 or pts.shape[1] < 3:
                return
            pts = pts[:, :3]
        vox = _voxels(pts, self.voxel)
        if self.prev is not None and vox and self.prev:
            inter = len(vox & self.prev)
            union = len(vox | self.prev)
            j = inter / max(union, 1)
            self.jaccards.append(j)
        self.prev = vox
        xy = np.linalg.norm(pts[:, :2], axis=1)
        near = pts[xy < self.near_r]
        if near.shape[0] > 50:
            self.z_stds.append(float(near[:, 2].std()))
        self.n += 1
        if self.n % 10 == 0:
            j_m = float(np.mean(self.jaccards)) if self.jaccards else float("nan")
            z_m = float(np.mean(self.z_stds)) if self.z_stds else float("nan")
            self.get_logger().info(
                f"frames={self.n} jaccard_mean={j_m:.3f} near_z_std_mean={z_m:.3f}"
            )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/livox/points")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--voxel", type=float, default=0.15)
    ap.add_argument("--near_r", type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = GhostCheck(args.topic, args.voxel, args.near_r)
    t0 = time.time()
    try:
        while rclpy.ok() and (time.time() - t0) < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        j = list(node.jaccards)
        z = list(node.z_stds)
        print(
            f"[ghost_summary] frames={node.n} "
            f"jaccard_mean={np.mean(j) if j else float('nan'):.3f} "
            f"jaccard_p10={np.percentile(j, 10) if j else float('nan'):.3f} "
            f"near_z_std_mean={np.mean(z) if z else float('nan'):.3f}"
        )
        # 粗门槛：静止常见 jaccard>0.35；行走若 <0.2 叠影风险高
        if j:
            jm = float(np.mean(j))
            if jm < 0.20:
                print("[ghost_summary] VERDICT=HIGH_GHOST_RISK（帧间重合过低，障碍/平地易误判）")
            elif jm < 0.35:
                print("[ghost_summary] VERDICT=MODERATE（行走可接受但需降速/滤波）")
            else:
                print("[ghost_summary] VERDICT=OK_LIKE_STATIC（相对清晰）")
        else:
            print("[ghost_summary] VERDICT=NO_DATA（检查话题/Play/bringup）")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
