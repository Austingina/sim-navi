"""Build a stable two-part collision proxy and package it into a new USDZ.

The reconstructed OBJ is split into:
  * the upward-facing connected surface containing the robot spawn (ground), and
  * sufficiently large non-horizontal components (walls/obstacles).

Only the ground component is smoothed.  Horizontal disconnected clutter such as
roofs, floating reconstruction fragments and table tops is deliberately omitted
from physics so it cannot trip the wheels.
"""

import argparse
import os
import shutil
import tempfile
import time
import zipfile

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils, Vt

from add_collision_to_usdz import load_obj, voxel_cluster


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)


def face_geometry(vertices, faces):
    tri = vertices[faces]
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    twice_area = np.linalg.norm(normal, axis=1)
    nz = np.divide(np.abs(normal[:, 2]), twice_area,
                   out=np.zeros_like(twice_area), where=twice_area > 1e-9)
    return nz, twice_area


def vertex_components(n_vertices, faces):
    """Return connected-component labels using only edges in ``faces``."""
    edges = np.concatenate((faces[:, (0, 1)], faces[:, (1, 2)],
                            faces[:, (2, 0)]), axis=0)
    rows = np.concatenate((edges[:, 0], edges[:, 1]))
    cols = np.concatenate((edges[:, 1], edges[:, 0]))
    graph = coo_matrix((np.ones(len(rows), dtype=np.uint8), (rows, cols)),
                       shape=(n_vertices, n_vertices)).tocsr()
    return connected_components(graph, directed=False, return_labels=True)[1]


def compact_mesh(vertices, faces):
    used, inverse = np.unique(faces.reshape(-1), return_inverse=True)
    return vertices[used].copy(), inverse.reshape(-1).astype(np.int32)


def make_heightfield(vertices, face_indices, cell, fill_radius, max_step,
                     spawn_xy):
    """Convert a floor mesh to a strict, single-valued 2.5-D height field.

    The lowest sample in each XY cell wins, which rejects roofs and upper
    surfaces.  Missing cells are filled only over a short distance.  Quads
    spanning a height discontinuity are not triangulated.
    """
    faces = face_indices.reshape(-1, 3)
    samples = np.concatenate((vertices, vertices[faces].mean(axis=1)), axis=0)
    lower = np.floor(samples[:, :2].min(axis=0) / cell) * cell
    upper = np.ceil(samples[:, :2].max(axis=0) / cell) * cell
    shape_xy = np.floor((upper - lower) / cell).astype(np.int64) + 1
    nx, ny = map(int, shape_xy)
    ij = np.rint((samples[:, :2] - lower) / cell).astype(np.int64)
    ij[:, 0] = np.clip(ij[:, 0], 0, nx - 1)
    ij[:, 1] = np.clip(ij[:, 1], 0, ny - 1)
    flat = ij[:, 0] * ny + ij[:, 1]
    height = np.full(nx * ny, np.inf, dtype=np.float64)
    np.minimum.at(height, flat, samples[:, 2])
    height = height.reshape(nx, ny)

    known = np.isfinite(height)
    distance, nearest = ndimage.distance_transform_edt(
        ~known, return_distances=True, return_indices=True)
    valid = distance <= fill_radius / cell
    filled = height[tuple(nearest)]
    # A median removes isolated low samples; a light Gaussian pass makes wheel
    # contact normals continuous.  Invalid areas remain invalid afterwards.
    filtered = ndimage.median_filter(filled, size=3, mode="nearest")
    filtered = ndimage.gaussian_filter(filtered, sigma=0.7, mode="nearest")

    grid_index = np.arange(nx * ny, dtype=np.int64).reshape(nx, ny)
    a = grid_index[:-1, :-1]
    b = grid_index[1:, :-1]
    c = grid_index[1:, 1:]
    d = grid_index[:-1, 1:]
    quad_valid = (valid[:-1, :-1] & valid[1:, :-1]
                  & valid[1:, 1:] & valid[:-1, 1:])
    z4 = np.stack((filtered[:-1, :-1], filtered[1:, :-1],
                   filtered[1:, 1:], filtered[:-1, 1:]), axis=0)
    quad_valid &= np.ptp(z4, axis=0) <= max_step
    q = np.stack((a[quad_valid], b[quad_valid], c[quad_valid],
                  d[quad_valid]), axis=1)
    out_faces = np.concatenate((q[:, (0, 1, 2)], q[:, (0, 2, 3)]), axis=0)

    gx = lower[0] + np.arange(nx) * cell
    gy = lower[1] + np.arange(ny) * cell
    xx, yy = np.meshgrid(gx, gy, indexing="ij")
    out_vertices = np.column_stack((xx.ravel(), yy.ravel(), filtered.ravel()))
    out_vertices, out_indices = compact_mesh(out_vertices, out_faces)

    # Height discontinuities can create several islands.  Keep only the island
    # reachable from the robot spawn.
    labels = vertex_components(len(out_vertices), out_indices.reshape(-1, 3))
    delta = out_vertices[:, :2] - np.asarray(spawn_xy)
    seed = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
    keep = np.all(labels[out_indices.reshape(-1, 3)] == labels[seed], axis=1)
    out_vertices, out_indices = compact_mesh(
        out_vertices, out_indices.reshape(-1, 3)[keep])
    print(f"[高度场] cell={cell:g}m fill={fill_radius:g}m "
          f"maxStep={max_step:g}m -> {len(out_vertices)} 顶点 / "
          f"{len(out_indices)//3} 三角面")
    return out_vertices.astype(np.float32), out_indices


def extract_proxies(vertices, face_indices, floor_cos, spawn, spawn_radius,
                    obstacle_cos, min_obstacle_faces):
    faces = face_indices.reshape(-1, 3)
    nz, twice_area = face_geometry(vertices, faces)

    floor_faces = faces[(nz >= floor_cos) & (twice_area > 1e-9)]
    labels = vertex_components(len(vertices), floor_faces)

    spawn = np.asarray(spawn, dtype=np.float64)
    delta = vertices.astype(np.float64) - spawn
    candidates = np.flatnonzero(
        (delta[:, 0] ** 2 + delta[:, 1] ** 2 <= spawn_radius ** 2)
        & (np.abs(delta[:, 2]) <= 0.5))
    if len(candidates) == 0:
        raise RuntimeError("出生点附近未找到地面顶点，请检查 --spawn-x/y/z")
    seed = candidates[np.argmin(np.linalg.norm(delta[candidates], axis=1))]
    ground_label = labels[seed]
    ground_faces = floor_faces[np.all(labels[floor_faces] == ground_label, axis=1)]
    ground_v, ground_f = compact_mesh(vertices, ground_faces)

    # Keep sides of walls/objects, but reject every horizontal disconnected
    # surface.  Small steep fragments are removed component-wise.
    steep = faces[(nz <= obstacle_cos) & (twice_area > 1e-9)]
    steep_labels = vertex_components(len(vertices), steep)
    face_labels = steep_labels[steep[:, 0]]
    counts = np.bincount(face_labels, minlength=int(face_labels.max()) + 1)
    obstacle_faces = steep[counts[face_labels] >= min_obstacle_faces]
    obstacle_v, obstacle_f = compact_mesh(vertices, obstacle_faces)

    print(f"[分离] 地面: {len(ground_v)} 顶点 / {len(ground_f)//3} 三角面, "
          f"范围 {ground_v.min(0)} -> {ground_v.max(0)}")
    print(f"[分离] 竖直结构: {len(obstacle_v)} 顶点 / "
          f"{len(obstacle_f)//3} 三角面")
    return ground_v, ground_f, obstacle_v, obstacle_f


def define_collision_mesh(stage, path, vertices, face_indices, color, visible):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(vertices)))
    mesh.CreateFaceVertexIndicesAttr(
        Vt.IntArray.FromNumpy(np.ascontiguousarray(face_indices)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(
        np.full(len(face_indices) // 3, 3, dtype=np.int32)))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(Vt.Vec3fArray([
        Gf.Vec3f(*map(float, vertices.min(0))),
        Gf.Vec3f(*map(float, vertices.max(0))),
    ]))
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    if not visible:
        mesh.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()) \
        .CreateApproximationAttr().Set("none")


def write_proxy_layer(path, ground_v, ground_f, obstacle_v, obstacle_f, visible):
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    define_collision_mesh(stage, "/World/GroundCollision", ground_v, ground_f,
                          (0.15, 0.75, 0.25), visible)
    define_collision_mesh(stage, "/World/ObstacleCollision", obstacle_v,
                          obstacle_f, (0.85, 0.35, 0.12), visible)
    stage.GetRootLayer().Save()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in", dest="input_usdz", default=os.path.join(
        PROJECT, "assets/zhichengAB/zhichengAB-collision-smooth-v3.usdz"))
    parser.add_argument("--obj", default=os.path.join(
        PROJECT, "assets/zhichengAB/mesh-files/zhichengAB.obj"))
    parser.add_argument("--out", default=os.path.join(
        PROJECT, "assets/zhichengAB/zhichengAB-collision-driveable.usdz"))
    parser.add_argument("--spawn-x", type=float, default=0.0)
    parser.add_argument("--spawn-y", type=float, default=0.0)
    parser.add_argument("--spawn-z", type=float, default=-1.47)
    parser.add_argument("--spawn-radius", type=float, default=2.0)
    parser.add_argument("--floor-cos", type=float, default=0.7)
    parser.add_argument("--obstacle-cos", type=float, default=0.55)
    parser.add_argument("--min-obstacle-faces", type=int, default=100)
    parser.add_argument("--ground-cell", type=float, default=0.20)
    parser.add_argument("--ground-fill-radius", type=float, default=0.40)
    parser.add_argument("--ground-max-step", type=float, default=0.25)
    parser.add_argument("--obstacle-voxel", type=float, default=0.08)
    parser.add_argument("--visible", action="store_true")
    args = parser.parse_args()

    for path in (args.input_usdz, args.obj):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
    if os.path.abspath(args.input_usdz) == os.path.abspath(args.out):
        raise ValueError("--out 必须是新文件，不能覆盖输入 USDZ")

    vertices, faces = load_obj(args.obj)
    meshes = extract_proxies(
        vertices, faces, args.floor_cos,
        (args.spawn_x, args.spawn_y, args.spawn_z), args.spawn_radius,
        args.obstacle_cos, args.min_obstacle_faces)
    del vertices, faces
    ground_v, ground_f, obstacle_v, obstacle_f = meshes

    ground_v, ground_f = make_heightfield(
        ground_v, ground_f, args.ground_cell, args.ground_fill_radius,
        args.ground_max_step, (args.spawn_x, args.spawn_y))
    obstacle_v, obstacle_f = voxel_cluster(
        obstacle_v, obstacle_f, args.obstacle_voxel)

    work = tempfile.mkdtemp(prefix="driveable_proxy_")
    try:
        print(f"[打包] 解包 {args.input_usdz}")
        with zipfile.ZipFile(args.input_usdz) as archive:
            archive.extractall(work)
        root_path = os.path.join(work, "default.usda")
        if not os.path.isfile(root_path):
            raise RuntimeError("输入 USDZ 中没有 default.usda")
        collision_path = os.path.join(work, "collision.usdc")
        write_proxy_layer(collision_path, ground_v, ground_f,
                          obstacle_v, obstacle_f, args.visible)

        root = Sdf.Layer.FindOrOpen(root_path)
        root.subLayerPaths = [p for p in root.subLayerPaths
                              if os.path.basename(p) != "collision.usdc"]
        root.subLayerPaths.append("./collision.usdc")
        root.Save()

        if os.path.exists(args.out):
            os.remove(args.out)
        print(f"[打包] 生成 {args.out}")
        started = time.time()
        if not UsdUtils.CreateNewUsdzPackage(root_path, args.out):
            raise RuntimeError("CreateNewUsdzPackage 失败")
        print(f"[完成] {os.path.getsize(args.out)/1e9:.2f} GB, "
              f"打包用时 {time.time()-started:.0f}s")
    finally:
        shutil.rmtree(work, ignore_errors=True)

    stage = Usd.Stage.Open(args.out)
    collision_prims = [p.GetPath().pathString for p in stage.Traverse()
                       if p.HasAPI(UsdPhysics.CollisionAPI)]
    print(f"[校验] 碰撞体: {collision_prims}")
    if "/World/GroundCollision" not in collision_prims \
            or "/World/ObstacleCollision" not in collision_prims:
        raise RuntimeError("输出校验失败")


if __name__ == "__main__":
    main()
