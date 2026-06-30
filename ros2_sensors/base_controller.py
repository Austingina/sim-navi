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

运行（系统 ROS2 Jazzy）：
    source /opt/ros/jazzy/setup.bash
    python3 base_controller.py
    # 键盘遥控: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

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
        self.declare_parameter("rate_hz", 50.0)

        self.pub = self.create_publisher(
            JointState, self.get_parameter("joint_cmd_topic").value, 10)
        self.sub = self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self.on_cmd, 10)
        self.js_sub = self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.on_joint_state,
            10,
        )

        self.vx = self.vy = self.wz = 0.0
        self.last_steer = {k: 0.0 for k in WHEELS}  # 零速时保持上次转向角
        self.hold_pos = {n: 0.0 for n in HOLD_JOINT_NAMES}
        self.hold_locked = False
        hz = self.get_parameter("rate_hz").value
        self.timer = self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info("swerve 控制器启动: /cmd_vel -> /joint_command")

    def on_cmd(self, msg: Twist):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z

    def on_joint_state(self, msg: JointState):
        # 首帧 joint_states 记录当前姿态，避免突然拽回零位
        if self.hold_locked:
            return
        for i, name in enumerate(msg.name):
            if name in self.hold_pos and i < len(msg.position):
                self.hold_pos[name] = msg.position[i]
        self.hold_locked = True
        self.get_logger().info("已锁定躯干/手臂保持位（来自 /joint_states 首帧）")

    def tick(self):
        steers, wheels = [], []
        moving = abs(self.vx) + abs(self.vy) + abs(self.wz) > 1e-4
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
            self.last_steer[key] = angle
            steers.append(angle)
            wheels.append(omega)

        hold = [self.hold_pos[n] for n in HOLD_JOINT_NAMES]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = STEER_NAMES + WHEEL_NAMES + HOLD_JOINT_NAMES
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
