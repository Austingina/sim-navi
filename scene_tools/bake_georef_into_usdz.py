#!/usr/bin/env python3
"""
把一份 georef（含 yaw_deg）直接焊进一个已有 usdz 的根层 customLayerData['georef']，
让地理配准随场景资产走。不重新处理网格，只改根层元数据后整包重打（含 .nurec，1.6GB，
需拷贝一遍，耗时以磁盘 IO 为主）。

georef 来源：读一个 json（默认 georef_square.json）。焊完再把同一份 json 覆盖写回，
保证 usdz customLayerData 与 gps_publisher 读的 json 完全一致（单一可信源）。

用法（在 scene_tools 目录，用带 pxr 的 python）：
  python3 bake_georef_into_usdz.py \
      --usdz ../assets/zhicheng-square/zhicheng-square-collision.usdz \
      --georef georef_square.json
"""
import argparse
import json
import os
import shutil
import tempfile
import time
import zipfile

from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils, Sdf

HERE = os.path.dirname(os.path.abspath(__file__))

GEOREF_KEYS = ("epsg", "utm_zone", "utm_north",
               "offset_x", "offset_y", "offset_z", "scale", "yaw_deg", "source")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usdz",
                    default=os.path.join(HERE, "..", "assets", "zhicheng-square",
                                         "zhicheng-square-collision.usdz"))
    ap.add_argument("--georef", default=os.path.join(HERE, "georef_square.json"))
    args = ap.parse_args()

    usdz = os.path.abspath(args.usdz)
    if not os.path.isfile(usdz):
        raise FileNotFoundError(usdz)
    with open(args.georef) as f:
        g = json.load(f)
    georef = {k: g[k] for k in GEOREF_KEYS if k in g}
    print(f"[georef] 将焊入: {georef}")

    # 临时目录放在同盘 workspace 下，避免 /tmp 空间/跨盘拷贝问题
    work = tempfile.mkdtemp(prefix="usdz_georef_", dir=os.path.join(HERE, ".."))
    try:
        print(f"[1/3] 解包 {os.path.relpath(usdz)} ...")
        with zipfile.ZipFile(usdz) as z:
            z.extractall(work)
            names = z.namelist()
        root = os.path.join(work, "default.usda")
        if not os.path.isfile(root):
            first = next(n for n in names
                         if n.lower().endswith((".usd", ".usda", ".usdc")))
            root = os.path.join(work, first)
        print(f"      根层: {os.path.relpath(root, work)}")

        print("[2/3] 写入 customLayerData['georef'] ...")
        layer = Sdf.Layer.FindOrOpen(root)
        cld = dict(layer.customLayerData)
        cld["georef"] = georef
        layer.customLayerData = cld
        layer.Save()

        print(f"[3/3] 重新打包 -> {os.path.relpath(usdz)} (含 .nurec, 需一会儿) ...")
        tmp_out = usdz + ".tmp"
        if os.path.exists(tmp_out):
            os.remove(tmp_out)
        t0 = time.time()
        if not UsdUtils.CreateNewUsdzPackage(root, tmp_out):
            raise RuntimeError("CreateNewUsdzPackage 失败")
        os.replace(tmp_out, usdz)
        print(f"[完成] {os.path.relpath(usdz)}  "
              f"({os.path.getsize(usdz)/1e9:.2f} GB, 用时 {time.time()-t0:.0f}s)")
    finally:
        shutil.rmtree(work, ignore_errors=True)

    # 覆盖写回 json，确保与 usdz 内一致
    with open(args.georef, "w") as f:
        json.dump(g, f, indent=2, ensure_ascii=False)

    # 校验：重开 usdz，确认 georef 焊入且碰撞体仍在
    print("[校验] 重新打开 usdz ...")
    stage = Usd.Stage.Open(usdz)
    baked = stage.GetRootLayer().customLayerData.get("georef")
    n_coll = sum(1 for p in stage.Traverse()
                 if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI))
    print(f"[校验] customLayerData['georef'] = {baked}")
    print(f"[校验] 带碰撞的 Mesh: {n_coll}")


if __name__ == "__main__":
    main()
