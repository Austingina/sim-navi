#!/usr/bin/env python3
"""
3 轮独立转向(swerve)底盘控制器：/cmd_vel -> /joint_command。

Isaac 没有 swerve 控制节点，这里用标准 swerve 运动学：
给定车体速度 (vx, vy, wz)，对每个轮子 i（位置 xi,yi）：
    vix = vx - wz*yi
    viy = vy + wz*xi
    转向角  steer_i = atan2(viy, vix)
    轮速    wheel_i = |v_i| / r
再做就近优化（转向角限制在 ±90°，必要时反转轮速），避免 180° 大转。

发布 sensor_msgs/JointState 到 /joint_command：
    position = [steer..., 0,0,0, hold...]   # 转向 + 躯干/手臂保持位
    velocity = [0,0,0, wheel..., 0,0,...]    # 轮关节用速度
（steer/arm/torso=位置驱动, wheel=速度驱动；未操控关节持续发保持位，避免晃动。）

运行（系统 ROS2 Humble）：
    source /opt/ros/humble/setup.bash
    python3 base_controller.py
    # 键盘遥控: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# 默认姿态文件（posture_ui.py 的"存为默认"写入；启动时加载为躯干/手臂保持位）。
POSTURE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "default_posture.json")


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _approach(cur, tgt, max_delta):
    """把 cur 朝 tgt 移动，单步最多 max_delta（限加速度用）。"""
    d = tgt - cur
    if d > max_delta:
        d = max_delta
    elif d < -max_delta:
        d = -max_delta
    return cur + d

# 轮子在 base_link 下的平面位置 (x 前, y 左)，取自 URDF steer_motor_joint 原点
WHEELS = {
    "1": (0.16897, 0.28),    # 前左
    "2": (0.16897, -0.28),   # 前右
    "3": (-0.32703, 0.0),    # 后中
}
WHEEL_RADIUS = 0.07          # 轮半径(米)，由轮网格估算，可调
WHEEL_SIGN = 1.0             # 若前进方向反了，改成 -1.0
STEER_NAMES = [f"steer_motor_joint{i}" for i in ("1", "2", "3")]
WHEEL_NAMES = [f"wheel_motor_joint{i}" for i in ("1", "2", "3")]
# 底盘以外、需要位置锁定的关节（与 URDF 一致）
HOLD_JOINT_NAMES = (
    [f"torso_joint{i}" for i in range(1, 5)]
    + [f"left_arm_joint{i}" for i in range(1, 8)]
    + ["left_gripper_finger_joint1", "left_gripper_finger_joint2"]
    + [f"right_arm_joint{i}" for i in range(1, 8)]
    + ["right_gripper_finger_joint1", "right_gripper_finger_joint2"]
)


class SwerveController(Node):
    def __init__(self):
        super().__init__("swerve_base_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_cmd_topic", "/joint_command")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("posture_topic", "/posture_command")
        # 注意：带 use_sim_time 时，timer 由 /clock 驱动，有效频率被 /clock(ISAAC_CLOCK_HZ,
        # 默认 20Hz)压着；设得比它高只会让 rclpy 反复追赶时钟、空烧 CPU。取 ≈ /clock 即可。
        self.declare_parameter("rate_hz", 30.0)
        # 限速 / 限加速 / 限转向速率：头重脚轻的底盘带着动量撞上地面颠簸就翻，
        # 这里给车体速度做上限 + 斜坡(限加速度) + 转向角限速，让轮子"骑过"而不是"撞进"小起伏。
        # 想放开(等价关闭)就把这些设很大。
        self.declare_parameter("max_vx", 0.6)          # m/s
        self.declare_parameter("max_vy", 0.6)          # m/s
        self.declare_parameter("max_wz", 0.8)          # rad/s
        self.declare_parameter("max_lin_accel", 0.4)   # m/s^2
        self.declare_parameter("max_ang_accel", 0.6)   # rad/s^2
        self.declare_parameter("max_steer_rate", 3.0)  # rad/s
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)
        self.max_lin_accel = float(self.get_parameter("max_lin_accel").value)
        self.max_ang_accel = float(self.get_parameter("max_ang_accel").value)
        self.max_steer_rate = float(self.get_parameter("max_steer_rate").value)

        self.pub = self.create_publisher(
            JointState, self.get_parameter("joint_cmd_topic").value, 10)
        self.sub = self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self.on_cmd, 10)
        # posture_ui.py 发来的手动姿态(躯干/手臂)，合并进 /joint_command 的保持位。
        # 这样"弯腰降重心"和底盘开车共用一个发布源，不会两个节点抢 /joint_command。
        self.posture_sub = self.create_subscription(
            JointState,
            self.get_parameter("posture_topic").value,
            self.on_posture,
            10,
        )

        self.tgt_vx = self.tgt_vy = self.tgt_wz = 0.0   # 目标(来自 /cmd_vel, 已夹取)
        self.vx = self.vy = self.wz = 0.0               # 当前(每 tick 按限加速度逼近目标)
        self.last_steer = {k: 0.0 for k in WHEELS}  # 零速时保持上次转向角
        self.hold_pos = {n: 0.0 for n in HOLD_JOINT_NAMES}
        self.hold_locked = False
        self._load_default_posture()   # 有默认姿态文件就直接用它当低重心保持位

        # /joint_states 只用来锁一次初始保持位：若已从文件加载(hold_locked)就完全不订阅；
        # 否则订阅、在首帧锁定后立即销毁，避免此后每帧白白反序列化整机关节状态。
        self.js_sub = None
        if not self.hold_locked:
            self.js_sub = self.create_subscription(
                JointState,
                self.get_parameter("joint_states_topic").value,
                self.on_joint_state,
                10,
            )

        # 复用一条 JointState：name 固定，只设一次(rclpy 给序列字段赋值会逐元素校验，
        # 每 tick 重建 name 实测占 Python 侧一半开销)。tick 里只更新 position/velocity。
        self._js = JointState()
        self._js.name = STEER_NAMES + WHEEL_NAMES + list(HOLD_JOINT_NAMES)

        hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / hz
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"swerve 控制器启动: /cmd_vel -> /joint_command "
            f"(限速 vx/vy≤{self.max_vx:g} wz≤{self.max_wz:g}, "
            f"限加速 lin={self.max_lin_accel:g} ang={self.max_ang_accel:g}, "
            f"转向≤{self.max_steer_rate:g} rad/s)")

    def on_cmd(self, msg: Twist):
        # 只夹取存为目标；实际速度在 tick 里按加速度上限斜坡逼近，避免速度突变冲进颠簸翻车。
        self.tgt_vx = _clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self.tgt_vy = _clamp(msg.linear.y, -self.max_vy, self.max_vy)
        self.tgt_wz = _clamp(msg.angular.z, -self.max_wz, self.max_wz)

    def on_joint_state(self, msg: JointState):
        # 首帧 joint_states 记录当前姿态，避免突然拽回零位
        if self.hold_locked:
            return
        for i, name in enumerate(msg.name):
            if name in self.hold_pos and i < len(msg.position):
                self.hold_pos[name] = msg.position[i]
        self.hold_locked = True
        self.get_logger().info("已锁定躯干/手臂保持位（来自 /joint_states 首帧）")
        # 锁定完成，此后不再需要 /joint_states：销毁订阅，省掉每帧整机关节反序列化。
        if self.js_sub is not None:
            self.destroy_subscription(self.js_sub)
            self.js_sub = None

    def _load_default_posture(self):
        """启动时加载 default_posture.json 到保持位并锁定（不再被 /joint_states 首帧直立姿态覆盖），
        使机器人一开始就摆成保存好的低重心姿态。文件不存在则维持原行为(锁首帧)。"""
        if not os.path.isfile(POSTURE_FILE):
            return
        try:
            with open(POSTURE_FILE) as f:
                data = json.load(f)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"读取 {POSTURE_FILE} 失败: {e}")
            return
        n = 0
        for name, pos in data.items():
            if name in self.hold_pos:
                self.hold_pos[name] = float(pos)
                n += 1
        self.hold_locked = True
        self.get_logger().info(f"已加载默认姿态 {os.path.basename(POSTURE_FILE)}：{n} 个关节(低重心)")

    def on_posture(self, msg: JointState):
        # 手动姿态(posture_ui.py)覆盖保持位：躯干弯下 -> 重心降低 -> 更不易翻。
        n = 0
        for i, name in enumerate(msg.name):
            if name in self.hold_pos and i < len(msg.position):
                self.hold_pos[name] = msg.position[i]
                n += 1
        # 收到手动姿态后就别再被 /joint_states 首帧覆盖了。
        self.hold_locked = True
        if n and not getattr(self, "_posture_seen", False):
            self._posture_seen = True
            self.get_logger().info(f"接管姿态：/posture_command 覆盖 {n} 个躯干/手臂关节保持位")

    def tick(self):
        # 限加速度：把当前速度按 max_*_accel*dt 斜坡逼近目标，再跑运动学。
        self.vx = _approach(self.vx, self.tgt_vx, self.max_lin_accel * self.dt)
        self.vy = _approach(self.vy, self.tgt_vy, self.max_lin_accel * self.dt)
        self.wz = _approach(self.wz, self.tgt_wz, self.max_ang_accel * self.dt)

        steers, wheels = [], []
        moving = abs(self.vx) + abs(self.vy) + abs(self.wz) > 1e-4
        max_steer_step = self.max_steer_rate * self.dt
        for key in ("1", "2", "3"):
            xi, yi = WHEELS[key]
            vix = self.vx - self.wz * yi
            viy = self.vy + self.wz * xi
            speed = math.hypot(vix, viy)
            if moving and speed > 1e-6:
                angle = math.atan2(viy, vix)
            else:
                angle = self.last_steer[key]   # 保持上次角度
            omega = speed / WHEEL_RADIUS * WHEEL_SIGN
            # 就近优化：转向角超过 ±90° 时翻转，并反转轮速
            diff = self._wrap(angle - self.last_steer[key])
            if abs(diff) > math.pi / 2:
                angle = self._wrap(angle + math.pi)
                omega = -omega
            # 转向角限速：每 tick 最多转 max_steer_rate*dt，避免急打方向导致侧向猛蹭/横向冲击。
            step = _clamp(self._wrap(angle - self.last_steer[key]),
                          -max_steer_step, max_steer_step)
            angle = self._wrap(self.last_steer[key] + step)
            self.last_steer[key] = angle
            steers.append(angle)
            wheels.append(omega)

        hold = [self.hold_pos[n] for n in HOLD_JOINT_NAMES]
        js = self._js                       # 复用同一条消息，name 已在 __init__ 设好
        js.header.stamp = self.get_clock().now().to_msg()
        js.position = steers + [0.0, 0.0, 0.0] + hold
        js.velocity = [0.0, 0.0, 0.0] + wheels + [0.0] * len(HOLD_JOINT_NAMES)
        self.pub.publish(js)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))


def main():
    rclpy.init()
    node = SwerveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
