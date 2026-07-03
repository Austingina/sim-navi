#!/usr/bin/env python3
"""
用「共同地标对应点」求新场景 -> 旧场景 的 2D 刚体/相似变换 (Umeyama)，
再结合旧场景已知 georef，算出新场景 gps_publisher 需要的
    yaw_deg / scale / offset_x / offset_y (spawn 设 0)。

原理：
    old_world = s * R(theta) * new_world + t          # 两场景是同一片区域的两次重建
    UTM       = old_world + offset_old                # 旧场景已配准, yaw_old=0, scale=1
  代入 new_world = spawn + odom (odom 系原点在出生点、与世界轴对齐)：
    UTM = s*R*odom + ( s*R*spawn + t + offset_old )
  对上 gps_publisher: UTM = scale*R(yaw)*odom + origin, origin = offset_json + spawn_param
  => scale=s, yaw=theta, 令 spawn_param=0 则 offset_json = s*R*spawn + t + offset_old
"""
import json
import math
import os

# ---- 输入：同一批真实地标在两个场景里的世界 (x, y) ----
# 顺序必须一一对应！
OLD = [(4.9999, 0.00195), (65.4, 0.02), (55.6, 9.2)]
NEW = [(1.0, -4.6), (-0.1, -65.4), (9.0, -55.7)]

# 机器人在【新场景】里的出生世界坐标 (Isaac spawn，odom=0 的地方)
SPAWN = (5.0, 0.0)

# 旧场景 georef（局部 + offset = UTM）
_HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(_HERE, "georef.json")) as f:
    G = json.load(f)
OFF = (G["offset_x"], G["offset_y"])
ESTIMATE_SCALE = False   # 两次重建通常都是米制，scale 固定 1；设 True 可估尺度


def umeyama_2d(src, dst, estimate_scale=False):
    """求 dst ≈ s*R*src + t。返回 (theta_rad, s, tx, ty, rmse)。"""
    n = len(src)
    mx = (sum(p[0] for p in src) / n, sum(p[1] for p in src) / n)
    my = (sum(p[0] for p in dst) / n, sum(p[1] for p in dst) / n)
    a = b = var_src = 0.0
    for (sx, sy), (dx, dy) in zip(src, dst):
        sx -= mx[0]; sy -= mx[1]
        dx -= my[0]; dy -= my[1]
        a += sx * dx + sy * dy          # ~cos
        b += sx * dy - sy * dx          # ~sin
        var_src += sx * sx + sy * sy
    theta = math.atan2(b, a)
    s = (math.hypot(a, b) / var_src) if estimate_scale else 1.0
    ct, st = math.cos(theta), math.sin(theta)
    tx = my[0] - s * (ct * mx[0] - st * mx[1])
    ty = my[1] - s * (st * mx[0] + ct * mx[1])
    # 残差
    se = 0.0
    for (sx, sy), (dx, dy) in zip(src, dst):
        ex = s * (ct * sx - st * sy) + tx
        ey = s * (st * sx + ct * sy) + ty
        se += (ex - dx) ** 2 + (ey - dy) ** 2
    return theta, s, tx, ty, math.sqrt(se / n)


def main():
    theta, s, tx, ty, rmse = umeyama_2d(NEW, OLD, ESTIMATE_SCALE)
    # offset_json = t + offset_old；spawn 由 gps_publisher 用 R(yaw)/scale 单独处理
    off_x = tx + OFF[0]
    off_y = ty + OFF[1]

    print("=== new -> old rigid transform ===")
    print(f"  yaw   = {math.degrees(theta):.4f} deg")
    print(f"  scale = {s:.6f}")
    print(f"  t     = ({tx:.4f}, {ty:.4f})")
    print(f"  fit RMSE = {rmse:.4f} m  (三点越小越可信)")
    print()
    print("=== 新场景 georef（写入 georef_square.json）===")
    print(f"  yaw_deg  = {math.degrees(theta):.4f}")
    print(f"  scale    = {s:.6f}")
    print(f"  offset_x = {off_x:.6f}")
    print(f"  offset_y = {off_y:.6f}")
    print(f"  （gps_publisher 里 spawn_x/spawn_y 仍填新场景真实出生点，如 5/0）")

    out = {
        "epsg": G.get("epsg"),
        "utm_zone": G["utm_zone"],
        "utm_north": G["utm_north"],
        "offset_x": off_x,
        "offset_y": off_y,
        "offset_z": G["offset_z"],
        "scale": s,
        "yaw_deg": math.degrees(theta),
        "source": "align_new_scene.py (3-pt to zhicheng)",
        "_note": "新区域折算到旧场景UTM系; offset=t+旧offset, gps_publisher读yaw_deg并对spawn/odom施加R(yaw)",
    }
    dst = os.path.join(_HERE, "georef_square.json")
    with open(dst, "w") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f"\n[OK] 写出 {dst}")


if __name__ == "__main__":
    main()
