#!/usr/bin/env python3
"""把 default_posture.json(弧度)烘焙进场景 USD 的关节：驱动目标位 + 初始状态。

作用：机器人一 Play 就以保存好的低重心姿态【出生并保持】，即使不跑 base_controller
(纯物理/Play 也稳)。base_controller 若同时在跑，命令的是同一姿态，一致不冲突。

⚠️ 单位：/joint_states(以及 default_posture.json) 是【弧度】，而 USD Physics 的角驱动
   drive:angular:physics:targetPosition 和 state:angular:physics:position 用【度】。
   本脚本自动 rad->deg，别手改数字进 USD(会差 57 倍)。

用法：
  python3 apply_posture_to_usd.py [--posture <json>] [scene1.usd scene2.usd ...]
  不指定场景时默认改 ../scene_seg_smooth.usd。
"""
import argparse
import json
import math
import os

from pxr import Usd

PROJECT = os.path.dirname(os.path.abspath(__file__))
JOINTS_PATH = "/World/r1_pro_with_gripper/joints"


def apply_to_scene(scene, posture):
    stage = Usd.Stage.Open(scene)
    if not stage:
        print(f"[跳过] 打不开 {scene}")
        return
    n = 0
    for name, rad in posture.items():
        prim = stage.GetPrimAtPath(f"{JOINTS_PATH}/{name}")
        if not prim.IsValid():
            continue
        deg = math.degrees(float(rad))
        did = False
        a = prim.GetAttribute("drive:angular:physics:targetPosition")
        if a and a.IsValid():
            a.Set(deg)
            did = True
        s = prim.GetAttribute("state:angular:physics:position")
        if s and s.IsValid():
            s.Set(deg)
            did = True
        if did:
            n += 1
    stage.GetRootLayer().Save()
    print(f"[OK] {os.path.basename(scene)}: 写入 {n} 个关节 targetPosition+state(已转为度)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--posture",
                    default=os.path.join(PROJECT, "..", "ros2_sensors", "default_posture.json"))
    ap.add_argument("scenes", nargs="*")
    args = ap.parse_args()

    with open(args.posture) as f:
        posture = json.load(f)
    print(f"[posture] 从 {os.path.relpath(args.posture)} 读入 {len(posture)} 个关节(弧度)")

    scenes = args.scenes or [os.path.join(PROJECT, "..", "scene_seg_smooth.usd")]
    for sc in scenes:
        apply_to_scene(sc, posture)


if __name__ == "__main__":
    main()
