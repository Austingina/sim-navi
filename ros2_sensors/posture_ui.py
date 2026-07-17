#!/usr/bin/env python3
"""滑条 UI：手动摆机器人躯干/手臂姿态（用来"弯腰"降低重心，减小翻车），并可存为默认。

发布 sensor_msgs/JointState 到 /posture_command（只含躯干+双臂关节的目标位置）。
本身**不直接**发 /joint_command —— 由 base_controller.py 订阅 /posture_command，
把这些姿态合并进它本来就在发的 /joint_command（转向/轮速 + 保持位）。这样"弯腰降重心"
和底盘开车共用一个发布源、不会两个节点抢，压低重心后还能照常 /cmd_vel 开车。

按钮：
  读取机器人当前姿态  —— 把滑条同步到 /joint_states 里机器人实际的当前角度
  存为默认            —— 把当前滑条角度写进 default_posture.json；base_controller 启动时
                        会自动加载它当保持位 -> 以后一开机就是这个低重心姿态
  全部归零            —— 滑条回 0

依赖：python3-tk（tkinter）+ 一个有显示的机器（比如你跑 rviz 那台）；须同时在跑 base_controller。
运行：source /opt/ros/humble/setup.bash && python3 posture_ui.py
"""
import json
import os
import tkinter as tk

import rclpy
from sensor_msgs.msg import JointState

TOPIC = "/posture_command"
POSTURE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "default_posture.json")

# (关节名, 下限, 上限, 初值)  —— 限位取自 r1_pro_with_gripper.urdf
JOINTS = [
    ("torso_joint1", -1.1345, 1.8326, 0.0),
    ("torso_joint2", -2.7925, 2.5307, 0.0),
    ("torso_joint3", -1.8326, 1.5708, 0.0),
    ("torso_joint4", -3.0543, 3.0543, 0.0),
    ("left_arm_joint1", -4.4506, 1.3090, 0.0),
    ("left_arm_joint2", -0.1745, 3.1416, 0.0),
    ("left_arm_joint3", -2.3562, 2.3562, 0.0),
    ("left_arm_joint4", -1.7453, 0.3491, 0.0),
    ("left_arm_joint5", -2.3562, 2.3562, 0.0),
    ("left_arm_joint6", -1.0472, 1.0472, 0.0),
    ("left_arm_joint7", -1.5708, 1.5708, 0.0),
    ("right_arm_joint1", -4.4506, 1.3090, 0.0),
    ("right_arm_joint2", -3.1416, 0.1745, 0.0),
    ("right_arm_joint3", -2.3562, 2.3562, 0.0),
    ("right_arm_joint4", -1.7453, 0.3491, 0.0),
    ("right_arm_joint5", -2.3562, 2.3562, 0.0),
    ("right_arm_joint6", -1.0472, 1.0472, 0.0),
    ("right_arm_joint7", -1.5708, 1.5708, 0.0),
]


def _load_saved():
    """读 default_posture.json（若有），作为滑条初值。"""
    if not os.path.isfile(POSTURE_FILE):
        return {}
    try:
        with open(POSTURE_FILE) as f:
            return json.load(f)
    except Exception:  # noqa: BLE001
        return {}


def main():
    rclpy.init()
    node = rclpy.create_node("posture_ui")
    pub = node.create_publisher(JointState, TOPIC, 10)

    latest_js = {}   # /joint_states 最新角度：name -> pos

    def on_js(msg: JointState):
        for i, nm in enumerate(msg.name):
            if i < len(msg.position):
                latest_js[nm] = msg.position[i]

    node.create_subscription(JointState, "/joint_states", on_js, 10)

    saved = _load_saved()
    names = [nm for nm, _, _, _ in JOINTS]

    root = tk.Tk()
    root.title("Posture UI -> /posture_command  (弯腰降重心)")
    tk.Label(root, text="拖动躯干关节让机器人前倾/下蹲降低重心；须同时运行 base_controller。",
             anchor="w").pack(fill="x", padx=6, pady=4)

    tkvars = {}
    for name, lo, hi, init in JOINTS:
        row = tk.Frame(root)
        row.pack(fill="x", padx=6)
        tk.Label(row, text=name, width=16, anchor="w").pack(side="left")
        var = tk.DoubleVar(value=float(saved.get(name, init)))
        tk.Scale(row, variable=var, from_=lo, to=hi, resolution=0.01,
                 orient="horizontal", length=380).pack(side="left", fill="x", expand=True)
        tkvars[name] = var

    status = tk.Label(root, text=f"发布 {TOPIC} @20Hz"
                      + ("（已载入 default_posture.json）" if saved else ""), anchor="w")

    def reset():
        for nm, _, _, init in JOINTS:
            tkvars[nm].set(init)

    def read_current():
        # 把滑条同步到机器人实际当前角度（/joint_states）
        n = 0
        for nm in names:
            if nm in latest_js:
                tkvars[nm].set(round(float(latest_js[nm]), 4))
                n += 1
        status.config(text=f"已从 /joint_states 读取 {n} 个关节的当前角度")

    def save_default():
        data = {nm: round(float(tkvars[nm].get()), 5) for nm in names}
        with open(POSTURE_FILE, "w") as f:
            json.dump(data, f, indent=2)
        status.config(text=f"已存为默认 -> {POSTURE_FILE}（base_controller 下次启动自动加载）")
        node.get_logger().info(f"已写入默认姿态 {POSTURE_FILE}")

    btns = tk.Frame(root)
    btns.pack(pady=6)
    tk.Button(btns, text="读取机器人当前姿态", command=read_current).pack(side="left", padx=4)
    tk.Button(btns, text="存为默认", command=save_default).pack(side="left", padx=4)
    tk.Button(btns, text="全部归零", command=reset).pack(side="left", padx=4)
    status.pack(fill="x", padx=6)

    def tick():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(tkvars[nm].get()) for nm in names]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(50, tick)   # 20Hz

    root.after(50, tick)
    root.protocol("WM_DELETE_WINDOW", root.quit)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
