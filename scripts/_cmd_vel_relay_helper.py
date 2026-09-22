#!/usr/bin/env python3
"""系统 Python 3.10 中继：订阅 /cmd_vel，向 stdout 打印 CMD 行供 Lab play 读取。

由 lab_play_go2_cmd_vel.py 在无本进程 rclpy 时启动；勿单独当业务入口。
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist


def main() -> None:
    topic = sys.argv[1] if len(sys.argv) > 1 else "/cmd_vel"
    rclpy.init()
    node = rclpy.create_node("lab_go2_cmd_vel_relay")
    state = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "t": 0.0}

    def cb(msg: Twist) -> None:
        state["vx"] = float(msg.linear.x)
        state["vy"] = float(msg.linear.y)
        state["wz"] = float(msg.angular.z)
        state["t"] = time.time()

    node.create_subscription(Twist, topic, cb, 10)
    print("[relay] subscribed", topic, flush=True)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        print(
            f"CMD {state['vx']:.6f} {state['vy']:.6f} {state['wz']:.6f} {state['t']:.3f}",
            flush=True,
        )
        time.sleep(0.05)


if __name__ == "__main__":
    main()
