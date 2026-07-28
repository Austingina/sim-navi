"""
把 mesh-files 里的三角网格作为"碰撞体"加进已有的 3DGS usdz 场景。

背景:
  assets/zhichengAB/lcc-usdz-result/zhichengAB.usdz 里只有一个高斯泼溅体 (Volume, 负责好看的画面),
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
      --in  ../assets/zhichengAB/lcc-usdz-result/zhichengAB.usdz \
      --obj ../assets/zhichengAB/mesh-files/zhichengAB.obj \
      --out ../assets/zhichengAB/zhichengAB-collision.usdz

  --visible          让碰撞网格可见(灰色), 方便首次目视检查它是否和高斯对齐;
                     确认对齐后去掉该参数(默认隐藏), 重新生成即可。
  --approximation    碰撞近似: none(默认, 原始三角面, 适合静态建筑) /
                     meshSimplification(面数太多卡顿时降负载) / convexDecomposition
  --voxel            体素聚类去噪格子(米), 默认 0.04。低通去掉亚-voxel 的重建毛刺,
                     保留大于 voxel 的真实地形/坡度。地面太颠加大, 墙变薄调小。
  --smooth-iters     Taubin λ|μ 平滑轮数, 默认 5。在 voxel 之后再磨残余高频抖动,
                     反收缩设计【不损失缓坡/大结构】。0=关闭。
  --smooth-floor-only  只平滑地面(按法向识别), 墙/门/障碍物完全不动;
                     可放心把 --smooth-iters 开到 30~60 也不伤竖直结构。
  --floor-cos        地面判定阈值 |n·ẑ|, 默认 0.7(≈允许 45° 缓坡); 坡更陡调小。

地面去颠簸推荐(全局, 温和):
  python3 add_collision_to_usdz.py --voxel 0.04 --smooth-iters 5 \
      --out ../assets/zhichengAB/zhichengAB-collision-smooth.usdz

地面猛猛开大(只平地面, 墙门不动, 保坡度):
  python3 add_collision_to_usdz.py --voxel 0.035 --smooth-iters 40 \
      --smooth-floor-only --floor-cos 0.7 \
      --out ../assets/zhichengAB/zhichengAB-collision-smoother.usdz
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


def voxel_cluster(verts, faces, voxel):
    """按 voxel 大小的网格对顶点聚类(每格取格内顶点均值作代表)，重映射三角面、丢弃塌陷三角形。
    等价于在 voxel 尺度做低通/去噪：亚-voxel 的重建噪声(绊倒轮子的小尖刺)被并掉，
    大于 voxel 的真实地形(室外坡/坎/墙)原样保留。voxel 取 1~2cm，远低于雷达 0.4° 分辨率，
    对 SLAM 不可见。仅用于【碰撞】网格。"""
    t0 = time.time()
    v = verts.astype(np.float64)
    keys = np.floor(v / voxel).astype(np.int64)                 # 每个顶点落在哪个格
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)    # inv: 老顶点 -> 新顶点
    inv = inv.ravel()
    n_new = len(uniq)
    new_v = np.zeros((n_new, 3), dtype=np.float64)
    counts = np.zeros(n_new, dtype=np.int64)
    np.add.at(new_v, inv, v)                                    # 每格顶点求和
    np.add.at(counts, inv, 1)
    new_v /= counts[:, None]                                    # -> 均值作代表
    nf = inv[faces.reshape(-1, 3)]                              # 面重映射到新索引
    good = ((nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2])
            & (nf[:, 0] != nf[:, 2]))                           # 丢弃塌陷(退化)三角形
    new_faces = nf[good].reshape(-1).astype(np.int32)
    print(f"[voxel] {voxel*100:.2f}cm 聚类去噪: 顶点 {len(verts)}->{n_new}, "
          f"三角形 {len(faces)//3}->{len(new_faces)//3}, 用时 {time.time()-t0:.1f}s")
    return new_v.astype(np.float32), new_faces


def _build_undirected_edges(faces, n_verts):
    """从三角面构建【去重】的无向边(双向)，用于均匀拉普拉斯的邻居聚合。
    返回 (src, dst, inv_deg)：src->dst 每条无向边各出现一次(两个方向)，
    inv_deg[i] = 1/deg(i)(孤立点记 1，避免除零)。拓扑固定，只需构建一次。"""
    f = faces.reshape(-1, 3)
    e = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]], axis=0)
    e = np.sort(e, axis=1)                       # 无向：小索引在前
    e = np.unique(e, axis=0)                     # 去掉相邻三角形共享的重复边
    src = np.concatenate([e[:, 0], e[:, 1]])     # 对称成双向
    dst = np.concatenate([e[:, 1], e[:, 0]])
    deg = np.bincount(src, minlength=n_verts).astype(np.float64)
    deg[deg == 0] = 1.0
    return src, dst, (1.0 / deg)[:, None]


def taubin_smooth(verts, faces, iters, lam=0.5, mu=-0.53):
    """Taubin λ|μ 平滑：交替一步正系数(λ>0)收缩 + 一步负系数(μ<0)反收缩，
    抵消普通拉普拉斯的整体缩水，因此【只磨掉高频抖动，缓坡/大结构原样保留】。
    纯 numpy 实现(均匀拉普拉斯)，不引入 scipy/open3d。仅用于【碰撞】网格。
    经验值 λ=0.5, μ=-0.53(|μ|>λ) 是 Taubin 论文推荐的稳定通带。"""
    if iters <= 0 or len(faces) == 0:
        return verts
    t0 = time.time()
    v = verts.astype(np.float64)
    src, dst, inv_deg = _build_undirected_edges(faces, len(v))

    def umbrella(p):
        # L(p)_i = mean_{j∈N(i)} p_j - p_i  (均匀权重拉普拉斯)
        acc = np.zeros_like(p)
        np.add.at(acc, src, p[dst])
        return acc * inv_deg - p

    for _ in range(iters):
        v += lam * umbrella(v)                   # 收缩一步
        v += mu * umbrella(v)                    # 反收缩一步(负系数)
    print(f"[taubin] 平滑 {iters} 轮 (λ={lam:g}, μ={mu:g}): 顶点 {len(v)} "
          f"(保坡度，去高频)，用时 {time.time()-t0:.1f}s")
    return v.astype(np.float32)


def smooth_floor_only(verts, faces, iters, lam=0.5, mu=-0.53, floor_cos=0.7):
    """只对【地面】做保坡度 Taubin 平滑，墙/门/障碍物一动不动。可以对地面下猛药
    (大 iters)而完全不牵连雷达要扫的竖直结构。纯 numpy，不加依赖。

    区分地面 vs 墙：按三角面法向。地面/坡面朝上 -> |n·ẑ| 大；墙/门框竖直 -> |n·ẑ|≈0。
    floor_cos=0.7 即接受与水平夹角 ≤45° 的面为地面，足以覆盖室内缓坡，同时把墙(≈90°)
    远远排除。坡越陡就把 floor_cos 调小(如 0.6=允许 53°)。

    保护墙根：只移动【内部地面顶点】(其相邻面【全是】地面)；地面与墙交界处的顶点被
    冻结当锚点 —— 这样地面内部磨得再狠，墙根那条缝也纹丝不动。"""
    if iters <= 0 or len(faces) == 0:
        return verts
    t0 = time.time()
    v = verts.astype(np.float64)
    f = faces.reshape(-1, 3)

    # 每个三角面的单位法向的 z 分量绝对值(碰撞网格法向朝向可能不一致，取 abs)
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    ln = np.linalg.norm(n, axis=1)
    ln[ln == 0] = 1.0
    nz = np.abs(n[:, 2] / ln)
    floor_face = nz >= floor_cos                 # 朝上 -> 地面候选

    ff = f[floor_face]                           # 地面面
    nf = f[~floor_face]                          # 非地面面(墙/门/障碍)
    is_floor_v = np.zeros(len(v), dtype=bool)
    touches_other = np.zeros(len(v), dtype=bool)
    if len(ff):
        is_floor_v[ff.reshape(-1)] = True
    if len(nf):
        touches_other[nf.reshape(-1)] = True
    # 内部地面顶点 = 属于地面面 且 不挨着任何非地面面(交界顶点冻结当锚点)
    movable = is_floor_v & ~touches_other

    if not movable.any() or len(ff) == 0:
        print(f"[floor] 未识别到可平滑的地面(floor_cos={floor_cos:g} 太严?)，跳过。")
        return verts

    # 邻接只用【地面面】的边构建 -> 平滑时不会把墙的顶点拉进平均
    e = np.concatenate([ff[:, [0, 1]], ff[:, [1, 2]], ff[:, [2, 0]]], axis=0)
    e = np.unique(np.sort(e, axis=1), axis=0)
    src = np.concatenate([e[:, 0], e[:, 1]])
    dst = np.concatenate([e[:, 1], e[:, 0]])
    deg = np.bincount(src, minlength=len(v)).astype(np.float64)
    deg[deg == 0] = 1.0
    inv_deg = (1.0 / deg)[:, None]
    mov = movable[:, None]                        # 只更新内部地面顶点

    def umbrella(p):
        acc = np.zeros_like(p)
        np.add.at(acc, src, p[dst])
        return acc * inv_deg - p

    for _ in range(iters):
        v += mov * (lam * umbrella(v))           # 收缩(仅地面内部)
        v += mov * (mu * umbrella(v))            # 反收缩
    print(f"[floor] 地面专属平滑 {iters} 轮 (floor_cos={floor_cos:g}, λ={lam:g}, μ={mu:g}): "
          f"地面面 {int(floor_face.sum())}/{len(f)}，移动顶点 {int(movable.sum())}/{len(v)}"
          f"(墙/门/交界冻结)，用时 {time.time()-t0:.1f}s")
    return v.astype(np.float32)


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
                    default=os.path.join(PROJECT_DIR, "..", "assets", "zhichengAB", "lcc-usdz-result", "zhichengAB.usdz"))
    ap.add_argument("--obj",
                    default=os.path.join(PROJECT_DIR, "..", "assets", "zhichengAB", "mesh-files", "zhichengAB.obj"))
    ap.add_argument("--out",
                    default=os.path.join(PROJECT_DIR, "..", "assets", "zhichengAB", "zhichengAB-collision.usdz"))
    ap.add_argument("--ply",
                    default=os.path.join(PROJECT_DIR, "..", "assets", "zhichengAB", "PLY", "point_cloud", "iteration_100", "point_cloud.ply"),
                    help="3DGS PLY，用于自动提取地理配准(offset/epsg/scale)并焊进新 usdz")
    ap.add_argument("--approximation", default="none",
                    choices=["none", "meshSimplification", "convexDecomposition", "convexHull"])
    ap.add_argument("--voxel", type=float, default=0.04,
                    help="对碰撞网格做体素聚类去噪的格子大小(米)，0=不做。默认 0.04(4cm)："
                         "并掉绊轮子的亚厘米重建噪声，保留真实地形/坡度，远小于雷达 0.4° "
                         "分辨率，SLAM 看不出差别。地面太颠可加大，墙/门框变薄则调小。")
    ap.add_argument("--smooth-iters", type=int, default=5,
                    help="Taubin λ|μ 平滑迭代轮数，0=不平滑。默认 5：在 voxel 去噪之后再磨掉"
                         "残余高频抖动，且【保留缓坡/大结构】(反收缩不缩水)。3~8 之间调。")
    ap.add_argument("--smooth-lambda", type=float, default=0.5,
                    help="Taubin 正向(收缩)系数 λ，默认 0.5。")
    ap.add_argument("--smooth-mu", type=float, default=-0.53,
                    help="Taubin 反向(反收缩)系数 μ，默认 -0.53(需 |μ|>λ 才不缩水)。")
    ap.add_argument("--smooth-floor-only", action="store_true",
                    help="只对地面(按法向识别)做 Taubin 平滑，墙/门/障碍物完全不动。"
                         "可以放心把 --smooth-iters 开很大(如 30~60)也不伤竖直结构。")
    ap.add_argument("--floor-cos", type=float, default=0.7,
                    help="地面判定阈值 |n·ẑ|：≥ 该值的三角面算地面。0.7≈允许 45° 缓坡；"
                         "坡更陡就调小(0.6≈53°)。仅配合 --smooth-floor-only 生效。")
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
        if args.voxel > 0:
            verts, faces = voxel_cluster(verts, faces, args.voxel)
        if args.smooth_iters > 0:
            if args.smooth_floor_only:
                verts = smooth_floor_only(verts, faces, args.smooth_iters,
                                          args.smooth_lambda, args.smooth_mu,
                                          args.floor_cos)
            else:
                verts = taubin_smooth(verts, faces, args.smooth_iters,
                                      args.smooth_lambda, args.smooth_mu)
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
