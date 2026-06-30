#!/usr/bin/env python3
"""
给 r1_pro 加 ROS 2 控制入口（在 Isaac Sim Script Editor 里跑）。

建一个通用控制图：
  ROS2 Subscribe JointState(/joint_command) -> Isaac Articulation Controller

之后任何关节都能通过往 /joint_command 发 sensor_msgs/JointState 来控制：
  - 手臂 / 躯干 / 夹爪 / 转向(steer)  -> 用 position 字段（位置驱动）
  - 轮子(wheel)                       -> 用 velocity 字段（速度驱动）
同一条消息里可同时带 position 和 velocity：每个关节按自己的驱动模式
取对应字段（已确认 steer/arm=位置驱动, wheel=速度驱动），互不冲突。

底盘是 3 轮独立转向(swerve)，cmd_vel->关节命令的运动学在
base_controller.py 里（单独的 ROS2 节点）。

本脚本还会给关节配置 Drive 刚度/阻尼，减少未操控部位的晃动。
"""
import omni.usd
from pxr import Sdf, UsdPhysics
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

ROBOT_PRIM = "/World/r1_pro_with_gripper"   # 与 setup_sensors.py 保持一致
BASE_LINK = ROBOT_PRIM + "/base_link"        # PhysX articulation 根（URDF 导入后在此）
JOINT_CMD_TOPIC = "/joint_command"
CONTROL_GRAPH = "/ActionGraph_control"

# 位置驱动关节刚度（steer / 躯干 / 手臂 / 夹爪）
POS_STIFFNESS = 1e5
POS_DAMPING = 1e3
STEER_STIFFNESS = 5e4
STEER_DAMPING = 5e2
# 速度驱动轮子：低刚度 + 阻尼，跟随 velocity 命令
WHEEL_DAMPING = 1e3
MAX_FORCE = 1e7


def _apply_pos_drive(joint_prim, stiffness, damping, target=0.0):
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
    else:
        return False
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(MAX_FORCE)
    drive.CreateTargetPositionAttr(target)
    return True


def _apply_wheel_drive(joint_prim):
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(WHEEL_DAMPING)
    drive.CreateMaxForceAttr(MAX_FORCE)
    drive.CreateTargetVelocityAttr(0.0)
    return True


def stiffen_joint_drives(stage, robot_prim_path):
    """给 articulation 关节加 Drive，Play 前运行。"""
    prefix = robot_prim_path.rstrip("/") + "/"
    n_pos = n_wheel = 0
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith(prefix):
            continue
        name = prim.GetName()
        if "wheel_motor_joint" in name:
            if _apply_wheel_drive(prim):
                n_wheel += 1
        elif prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            stiff = STEER_STIFFNESS if "steer_motor_joint" in name else POS_STIFFNESS
            damp = STEER_DAMPING if "steer_motor_joint" in name else POS_DAMPING
            if _apply_pos_drive(prim, stiff, damp):
                n_pos += 1
    print(f"[OK] joint drives: {n_pos} position, {n_wheel} wheel velocity")


def _remove_graph_if_exists(stage, graph_path):
    """重复运行脚本时先删掉旧 Action Graph，避免 OmniGraphError。"""
    prim = stage.GetPrimAtPath(graph_path)
    if prim.IsValid():
        stage.RemovePrim(graph_path)
        print(f"[OK] removed existing graph: {graph_path}")


def setup():
    enable_extension("isaacsim.ros2.bridge")
    stage = omni.usd.get_context().get_stage()
    assert stage.GetPrimAtPath(ROBOT_PRIM).IsValid(), f"找不到 {ROBOT_PRIM}，请改 ROBOT_PRIM"
    assert stage.GetPrimAtPath(BASE_LINK).IsValid(), f"找不到 {BASE_LINK}，请检查 articulation 根路径"
    stiffen_joint_drives(stage, ROBOT_PRIM)
    _remove_graph_if_exists(stage, CONTROL_GRAPH)

    og.Controller.edit(
        {"graph_path": CONTROL_GRAPH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubJoint", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArtCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubJoint.inputs:topicName", JOINT_CMD_TOPIC),
                ("ArtCtrl.inputs:targetPrim", [Sdf.Path(BASE_LINK)]),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "SubJoint.inputs:execIn"),
                ("SubJoint.outputs:execOut", "ArtCtrl.inputs:execIn"),
                ("SubJoint.outputs:jointNames", "ArtCtrl.inputs:jointNames"),
                ("SubJoint.outputs:positionCommand", "ArtCtrl.inputs:positionCommand"),
                ("SubJoint.outputs:velocityCommand", "ArtCtrl.inputs:velocityCommand"),
                ("SubJoint.outputs:effortCommand", "ArtCtrl.inputs:effortCommand"),
                ("Ctx.outputs:context", "SubJoint.inputs:context"),
            ],
        },
    )
    print("[OK] control graph created. Publish JointState to", JOINT_CMD_TOPIC, "to drive joints")
    print("Articulation root:", BASE_LINK)


if __name__ == "__main__":
    setup()
