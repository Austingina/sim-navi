#!/usr/bin/env python3
"""
从 3DGS 重建的 point_cloud.ply 头部自动提取地理配准信息，生成权威的 georef.json。

为什么：
  重建工具(L2Pro)在 PLY 头里写了把“局部坐标 -> 真实投影坐标(UTM)”的全部信息：
    offsetx/y/z   局部原点对应的真实 UTM 坐标   (local + offset = UTM)
    scalex/y/z    局部单位 -> UTM 米 的尺度       (本场景 = 1，即 1 单位 = 1 UTM 米)
    shiftx/y/z    额外平移                         (本场景 = 0)
    epsg          投影 CRS                         (32649 = WGS84/UTM zone 49N)
  这些就是“标定结果”，无需两点手工标定。本脚本把它落成一个 JSON，
  让 scene.usd / gps_publisher.py 直接读，全链路单一可信源。

用法：
  python3 make_georef.py \
      --ply ../zhicheng/point_cloud/iteration_100/point_cloud.ply \
      --out lcc-usdz-result/georef.json

纯标准库实现，无需 numpy / pxr。
"""
import argparse
import json
import os

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))


def read_ply_georef(ply_path):
    """读 PLY ascii 头里的 georef 注释，返回 dict。"""
    fields = {}
    with open(ply_path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                break
            s = line.decode("ascii", "ignore").strip()
            if s == "end_header":
                break
            if not s.startswith("comment "):
                continue
            parts = s.split()
            if len(parts) >= 3:
                key = parts[1].lower()
                try:
                    fields[key] = float(parts[2])
                except ValueError:
                    fields[key] = parts[2]
    return fields


def epsg_to_utm(epsg):
    """326xx -> (zone=xx, north=True); 327xx -> (zone=xx, north=False)。"""
    epsg = int(epsg)
    if 32601 <= epsg <= 32660:
        return epsg - 32600, True
    if 32701 <= epsg <= 32760:
        return epsg - 32700, False
    raise ValueError(f"非 UTM/WGS84 的 EPSG: {epsg}（本脚本只处理 326xx/327xx）")


def build_georef(fields):
    epsg = int(fields.get("epsg", 32649))
    zone, north = epsg_to_utm(epsg)
    # scale*：三轴一般一致，取 x；缺省 1.0
    scale = float(fields.get("scalex", 1.0))
    return {
        "epsg": epsg,
        "utm_zone": zone,
        "utm_north": north,
        "offset_x": float(fields.get("offsetx", 0.0)),
        "offset_y": float(fields.get("offsety", 0.0)),
        "offset_z": float(fields.get("offsetz", 0.0)),
        "shift_x": float(fields.get("shiftx", 0.0)),
        "shift_y": float(fields.get("shifty", 0.0)),
        "shift_z": float(fields.get("shiftz", 0.0)),
        "scale": scale,
        "source": fields.get("source", "unknown"),
        "local_bounds_min": [
            float(fields.get("minx", 0.0)),
            float(fields.get("miny", 0.0)),
            float(fields.get("minz", 0.0)),
        ],
        "local_bounds_max": [
            float(fields.get("maxx", 0.0)),
            float(fields.get("maxy", 0.0)),
            float(fields.get("maxz", 0.0)),
        ],
        "_note": "local + offset = UTM(EPSG); scale=1 表示 1 场景单位 = 1 UTM 米; "
                 "scene.usd/gps_publisher 直接读此文件，无需手工标定。",
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ply", default=os.path.join(
        PROJECT_DIR, "..", "zhicheng", "point_cloud", "iteration_100", "point_cloud.ply"))
    ap.add_argument("--out", default=os.path.join(
        PROJECT_DIR, "lcc-usdz-result", "georef.json"))
    args = ap.parse_args()

    if not os.path.isfile(args.ply):
        raise FileNotFoundError(args.ply)
    fields = read_ply_georef(args.ply)
    georef = build_georef(fields)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(georef, f, indent=2, ensure_ascii=False)

    print(f"[georef] 从 {os.path.relpath(args.ply)} 提取并写入 {os.path.relpath(args.out)}:")
    print(json.dumps(georef, indent=2, ensure_ascii=False))
    print(f"\n关键: EPSG={georef['epsg']} (UTM {georef['utm_zone']}"
          f"{'N' if georef['utm_north'] else 'S'}), scale={georef['scale']} "
          f"(=1 表示无尺度差), offset=({georef['offset_x']},{georef['offset_y']},{georef['offset_z']})")


if __name__ == "__main__":
    main()
