"""
把 mesh-files 里的三角网格作为"碰撞体"加进已有的 3DGS usdz 场景。

背景:
  assets/zhicheng/raw_l2pro/zhicheng-usd.usdz 里只有一个高斯泼溅体 (Volume, 负责好看的画面),
  没有任何可参与物理的几何。本脚本把同一坐标系下的 OBJ 网格写成一个隐藏的三角网格
  碰撞体, 挂进场景, 重新打包成一个自包含的新 usdz。机器人就能"撞到"环境, 同时看到
  的仍是高斯渲染。

做法:
  1. 解包原 usdz (它本质是 zip) 到临时目录, 拿到根层 default.usda / gauss.usda / .nurec
  2. 解析 OBJ, 把几何写成 collision.usdc, 加 PhysicsCollisionAPI + MeshCollisionAPI(none)
  3. 把 collision.usdc 作为 subLayer 挂到根层 default.usda 上
  4. UsdUtils.CreateNewUsdzPackage 跟随依赖(含 1.6GB 的 .nurec)重新打包成新 usdz

依赖: pip install usd-core numpy  (或用 Isaac 自带 python)

用法:
  python3 add_collision_to_usdz.py \
      --in  ../assets/zhicheng/raw_l2pro/zhicheng-usd.usdz \
      --obj ../assets/zhicheng/raw_l2pro/zhicheng-usd.obj \
      --out ../assets/zhicheng/zhicheng-usd-collision.usdz

  --visible          让碰撞网格可见(灰色), 方便首次目视检查它是否和高斯对齐;
                     确认对齐后去掉该参数(默认隐藏), 重新生成即可。
  --approximation    碰撞近似: none(默认, 原始三角面, 适合静态建筑) /
                     meshSimplification(面数太多卡顿时降负载) / convexDecomposition
"""

import argparse
import json
import os
import shutil
import tempfile
import time
import zipfile

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils, Vt, Gf, Sdf

import make_georef  # 同目录：从 PLY 读取地理配准

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))


def bake_georef(root_layer, ply_path):
    """从 PLY 读地理配准，写进根层 customLayerData['georef']，并落 georef.json。
    这样新 usdz 自带“标定结果”，gps_publisher.py 直接读 georef.json，无需手工标定。"""
    if not ply_path or not os.path.isfile(ply_path):
        print(f"[georef] 跳过：未找到 PLY({ply_path})")
        return
    georef = make_georef.build_georef(make_georef.read_ply_georef(ply_path))
    # 1) 写进 usdz 根层 customLayerData
    layer = Sdf.Layer.FindOrOpen(root_layer)
    cld = dict(layer.customLayerData)
    cld["georef"] = {
        "epsg": georef["epsg"], "utm_zone": georef["utm_zone"],
        "utm_north": georef["utm_north"], "offset_x": georef["offset_x"],
        "offset_y": georef["offset_y"], "offset_z": georef["offset_z"],
        "scale": georef["scale"], "source": georef["source"],
    }
    layer.customLayerData = cld
    layer.Save()
    # 2) 落 georef.json 到工具目录(随仓库跟踪)，供 gps_publisher.py / scene.usd 读。
    #    成品 usdz 在 gitignore 的 assets/ 下，不能把唯一可信源放那里。
    json_path = os.path.join(PROJECT_DIR, "georef.json")
    with open(json_path, "w") as f:
        json.dump(georef, f, indent=2, ensure_ascii=False)
    print(f"[georef] 已写入 usdz customLayerData + {os.path.relpath(json_path)} "
          f"(EPSG={georef['epsg']}, scale={georef['scale']}, offset=("
          f"{georef['offset_x']},{georef['offset_y']}))")


def load_obj(path):
    """快速解析 OBJ, 只取顶点 v 和三角面 f (支持多边形扇形三角化)。"""
    t0 = time.time()
    with open(path, "rb") as f:
        lines = f.read().split(b"\n")
    vx, vy, vz, fi = [], [], [], []
    for ln in lines:
        if not ln:
            continue
        if ln[0] == 118 and ln[1] == 32:          # 'v '
            p = ln.split()
            vx.append(float(p[1])); vy.append(float(p[2])); vz.append(float(p[3]))
        elif ln[0] == 102 and ln[1] == 32:        # 'f '
            p = ln.split()
            idx = [int(t.split(b"/")[0]) - 1 for t in p[1:]]
            for k in range(1, len(idx) - 1):
                fi.append(idx[0]); fi.append(idx[k]); fi.append(idx[k + 1])
    verts = np.empty((len(vx), 3), dtype=np.float32)
    verts[:, 0] = vx; verts[:, 1] = vy; verts[:, 2] = vz
    faces = np.asarray(fi, dtype=np.int32)
    print(f"[OBJ] 顶点 {len(verts)}, 三角形 {len(faces)//3}, 用时 {time.time()-t0:.1f}s")
    return verts, faces


def write_collision_layer(usdc_path, verts, faces, approximation, visible):
    """把网格几何写成一个独立 usdc 图层, 并加好碰撞 API。"""
    stage = Usd.Stage.CreateNew(usdc_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # 与根层同名 /World, sublayer 合成时会并入 gauss 同级
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    mesh = UsdGeom.Mesh.Define(stage, "/World/CollisionMesh")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(verts)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.ascontiguousarray(faces)))
    counts = np.full(len(faces) // 3, 3, dtype=np.int32)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(counts))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    lo = verts.min(axis=0); hi = verts.max(axis=0)
    mesh.CreateExtentAttr(Vt.Vec3fArray([
        Gf.Vec3f(*[float(x) for x in lo]), Gf.Vec3f(*[float(x) for x in hi])]))
    if visible:
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.6, 0.6, 0.6)]))
    else:
        # 隐藏: 不遮挡高斯画面, 但仍参与物理碰撞
        UsdGeom.Imageable(mesh).CreateVisibilityAttr(UsdGeom.Tokens.invisible)

    # 静态环境三角网格碰撞: approximation=none 才能让机器人进出房间/走廊
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(approximation)

    stage.GetRootLayer().Save()
    print(f"[USD] 碰撞网格 -> {usdc_path} (approximation={approximation}, "
          f"{'可见' if visible else '隐藏'})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="in_usdz",
                    default=os.path.join(PROJECT_DIR, "..", "assets", "zhicheng", "raw_l2pro", "zhicheng-usd.usdz"))
    ap.add_argument("--obj",
                    default=os.path.join(PROJECT_DIR, "..", "assets", "zhicheng", "raw_l2pro", "zhicheng-usd.obj"))
    ap.add_argument("--out",
                    default=os.path.join(PROJECT_DIR, "..", "assets", "zhicheng", "zhicheng-usd-collision.usdz"))
    ap.add_argument("--ply",
                    default=os.path.join(PROJECT_DIR, "..", "assets", "zhicheng", "raw_l2pro", "point_cloud.ply"),
                    help="3DGS PLY，用于自动提取地理配准(offset/epsg/scale)并焊进新 usdz")
    ap.add_argument("--approximation", default="none",
                    choices=["none", "meshSimplification", "convexDecomposition", "convexHull"])
    ap.add_argument("--visible", action="store_true",
                    help="碰撞网格可见(灰), 用于首次目视检查对齐; 默认隐藏")
    args = ap.parse_args()

    for p in (args.in_usdz, args.obj):
        if not os.path.isfile(p):
            raise FileNotFoundError(p)

    work = tempfile.mkdtemp(prefix="usdz_collision_")
    try:
        # 1) 解包原 usdz
        print(f"[1/4] 解包 {args.in_usdz} -> {work}")
        with zipfile.ZipFile(args.in_usdz) as z:
            z.extractall(work)
        root_layer = os.path.join(work, "default.usda")
        if not os.path.isfile(root_layer):
            # 个别 usdz 根层名不固定, 取压缩包内第一个 usd* 文件
            with zipfile.ZipFile(args.in_usdz) as z:
                first = next(n for n in z.namelist() if n.lower().endswith((".usd", ".usda", ".usdc")))
            root_layer = os.path.join(work, first)
        print(f"      根层: {os.path.basename(root_layer)}")

        # 2) 解析 OBJ + 写碰撞层
        print("[2/4] 解析 OBJ 并写碰撞网格 ...")
        verts, faces = load_obj(args.obj)
        coll_usdc = os.path.join(work, "collision.usdc")
        write_collision_layer(coll_usdc, verts, faces, args.approximation, args.visible)

        # 3) 把碰撞层挂为根层的 subLayer
        print("[3/4] 挂载 subLayer 到根层 ...")
        layer = Sdf.Layer.FindOrOpen(root_layer)
        subs = list(layer.subLayerPaths)
        if "./collision.usdc" not in subs:
            layer.subLayerPaths.append("./collision.usdc")
        layer.Save()

        # 3.5) 把地理配准(来自 PLY)焊进根层 customLayerData + 落 georef.json
        print("[3.5] 焊入地理配准 georef ...")
        bake_georef(root_layer, args.ply)

        # 4) 重新打包成自包含 usdz (跟随依赖, 含 .nurec)
        print(f"[4/4] 重新打包 -> {args.out} (含 1.6GB .nurec, 需要一会儿) ...")
        if os.path.exists(args.out):
            os.remove(args.out)
        t0 = time.time()
        ok = UsdUtils.CreateNewUsdzPackage(root_layer, args.out)
        if not ok:
            raise RuntimeError("CreateNewUsdzPackage 失败")
        size_gb = os.path.getsize(args.out) / 1e9
        print(f"[完成] {args.out}  ({size_gb:.2f} GB, 用时 {time.time()-t0:.0f}s)")
    finally:
        shutil.rmtree(work, ignore_errors=True)

    # 校验: 重新打开, 确认 mesh + 碰撞都在
    print("[校验] 重新打开输出文件 ...")
    stage = Usd.Stage.Open(args.out)
    n_mesh = n_coll = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            n_mesh += 1
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                n_coll += 1
    print(f"[校验] Mesh prim: {n_mesh}, 带碰撞的: {n_coll}")


if __name__ == "__main__":
    main()
