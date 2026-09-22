#!/usr/bin/env python3
"""阶段 C：Isaac 大学城场景内用 Go2 速度策略跟 /cmd_vel，并保留 Mid360（同进程可做 lidar_ghost_check）。

用 Isaac 自带 python 跑（不要 env_isaaclab）：

  ./run_isaacsim.sh 会污染；推荐：

  $ISAACSIM_PATH/python.sh scripts/isaac_go2_policy_cmd_vel.py \\
      --usd $PWD/scene_daxuecheng_go2.usd \\
      --policy .../exported/policy.pt \\
      --deploy-yaml .../params/deploy.yaml

另终端：
  ros2 topic pub --rate 10 /cmd_vel ...
  python3 scripts/lidar_ghost_check.py --topic /livox/points --seconds 20
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time

# Kit 可能把 --/xxx 传给脚本；先剥离，留给 SimulationApp
_kit_args = [a for a in sys.argv[1:] if a.startswith("--/")]
sys.argv = [sys.argv[0]] + [a for a in sys.argv[1:] if not a.startswith("--/")]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--usd", required=True, help="scene_daxuecheng_go2.usd 绝对路径")
    p.add_argument("--policy", required=True, help="TorchScript policy.pt / policy_traced.pt")
    p.add_argument("--deploy-yaml", required=True, help="unitree deploy.yaml（观测/动作缩放）")
    p.add_argument("--physics-hz", type=float, default=200.0)
    p.add_argument("--setup-sensors", action="store_true", default=True)
    p.add_argument("--no-setup-sensors", action="store_false", dest="setup_sensors")
    p.add_argument("--cmd-timeout", type=float, default=0.5)
    p.add_argument("--kp", type=float, default=25.0)
    p.add_argument("--kd", type=float, default=0.5)
    p.add_argument("--device", default="cuda:0")
    return p.parse_args()


ARGS = parse_args()
if not os.path.isabs(ARGS.usd) or not os.path.isfile(ARGS.usd):
    raise SystemExit(f"--usd must be absolute existing file: {ARGS.usd}")
if not os.path.isfile(ARGS.policy):
    raise SystemExit(f"--policy not found: {ARGS.policy}")
if not os.path.isfile(ARGS.deploy_yaml):
    raise SystemExit(f"--deploy-yaml not found: {ARGS.deploy_yaml}")

from isaacsim import SimulationApp  # noqa: E402

# 相机 RenderProduct 需要 viewport 更新；默认开（对齐 outdoor.rviz /zed）
_viewport = os.environ.get("ISAAC_VIEWPORT", "1") == "1"
simulation_app = SimulationApp(
    {"headless": True, "disable_viewport_updates": not _viewport}
)
print(
    f"[policy] disable_viewport_updates={not _viewport} "
    f"(ISAAC_VIEWPORT={os.environ.get('ISAAC_VIEWPORT', '1')})",
    flush=True,
)

import numpy as np  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

ROBOT_PRIM = "/World/go2/base"
# 扶正时在当前高度上再抬这么多，避免肚子还埋在路面里。
STANDUP_Z = float(os.environ.get("ISAAC_STANDUP_Z", "0.35"))
POSE_PIDFILE = "/tmp/go2_policy_pose_ctl.pid"
# 只在主循环里读。信号回调只许改这个字典。
_pose_cmd = {"standup": False, "origin": False}

# Lab 训练时 joint 顺序（与 deploy.yaml default_joint_pos 对齐）
LAB_JOINT_ORDER = [
    "FL_hip_joint",
    "FR_hip_joint",
    "RL_hip_joint",
    "RR_hip_joint",
    "FL_thigh_joint",
    "FR_thigh_joint",
    "RL_thigh_joint",
    "RR_thigh_joint",
    "FL_calf_joint",
    "FR_calf_joint",
    "RL_calf_joint",
    "RR_calf_joint",
]


def quat_wxyz_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def world_to_body(vec_w: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    r = quat_wxyz_to_rot_matrix(quat_wxyz)
    return r.T @ np.asarray(vec_w, dtype=np.float64)


class CmdVelRelay:
    """系统 python3 中继订阅 /cmd_vel（Isaac 进程常无可用 Humble rclpy）。"""

    def __init__(self, timeout: float) -> None:
        self.timeout = timeout
        self.vx = self.vy = self.wz = 0.0
        self._stamp = 0.0
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        helper = os.path.join(repo, "scripts", "_cmd_vel_relay_helper.py")
        dds = os.path.join(repo, "cyclonedds_localhost.xml")
        bash = f"""
env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1 \
  CYCLONEDDS_URI=file://{dds} \
  /bin/bash -lc 'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 {helper} /cmd_vel'
"""
        self._proc = subprocess.Popen(
            ["/bin/bash", "-lc", bash],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env={"HOME": os.path.expanduser("~"), "USER": os.environ.get("USER", ""), "PATH": "/usr/bin:/bin"},
        )

        def _reader() -> None:
            assert self._proc.stdout
            for line in self._proc.stdout:
                line = line.strip()
                if line.startswith("CMD "):
                    parts = line.split()
                    if len(parts) >= 5:
                        self.vx, self.vy, self.wz = float(parts[1]), float(parts[2]), float(parts[3])
                        self._stamp = float(parts[4])
                elif line:
                    print(f"[cmd_relay] {line}", flush=True)

        threading.Thread(target=_reader, daemon=True).start()
        print("[policy] cmd_vel relay started", flush=True)

    def read(self) -> tuple[float, float, float]:
        if time.time() - self._stamp > self.timeout:
            return 0.0, 0.0, 0.0
        return self.vx, self.vy, self.wz

    def close(self) -> None:
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except Exception:
                self._proc.kill()


def _on_pose_signal(signum, _frame) -> None:
    if signum == signal.SIGUSR1:
        _pose_cmd["origin"] = True
    elif signum == signal.SIGUSR2:
        _pose_cmd["standup"] = True


def _yaw_of(q: np.ndarray) -> float:
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def _upright_like_spawn(current_q: np.ndarray, spawn_q: np.ndarray) -> np.ndarray:
    """保留当前航向，滚转俯仰回到出生时的直立姿态。"""
    dyaw = _yaw_of(current_q) - _yaw_of(spawn_q)
    half = 0.5 * dyaw
    yaw_q = np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)
    out = _quat_mul(yaw_q, np.asarray(spawn_q, dtype=np.float64))
    n = float(np.linalg.norm(out))
    return out if n < 1e-8 else out / n


def load_deploy(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main() -> int:
    cfg = load_deploy(ARGS.deploy_yaml)
    step_dt = float(cfg.get("step_dt", 0.02))
    default_pos = np.asarray(cfg["default_joint_pos"], dtype=np.float32)
    act_scale = np.asarray(cfg["actions"]["JointPositionAction"]["scale"], dtype=np.float32)
    act_offset = np.asarray(cfg["actions"]["JointPositionAction"]["offset"], dtype=np.float32)
    obs_scales = {
        "base_ang_vel": np.asarray(cfg["observations"]["base_ang_vel"]["scale"], dtype=np.float32),
        "projected_gravity": np.asarray(cfg["observations"]["projected_gravity"]["scale"], dtype=np.float32),
        "velocity_commands": np.asarray(cfg["observations"]["velocity_commands"]["scale"], dtype=np.float32),
        "joint_pos_rel": np.asarray(cfg["observations"]["joint_pos_rel"]["scale"], dtype=np.float32),
        "joint_vel_rel": np.asarray(cfg["observations"]["joint_vel_rel"]["scale"], dtype=np.float32),
        "last_action": np.asarray(cfg["observations"]["last_action"]["scale"], dtype=np.float32),
    }

    print(f"[policy] opening {ARGS.usd}", flush=True)
    open_stage(ARGS.usd)
    while is_stage_loading():
        simulation_app.update()

    if ARGS.setup_sensors:
        setup_py = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ros2_sensors", "setup_sensors_go2.py")
        import importlib.util

        spec = importlib.util.spec_from_file_location("setup_sensors_go2", setup_py)
        mod = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(mod)
        print("[policy] running setup_sensors_go2", flush=True)
        mod.setup()
        for _ in range(5):
            simulation_app.update()

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / ARGS.physics_hz, rendering_dt=1.0 / ARGS.physics_hz)
    robot = Robot(prim_path=ROBOT_PRIM, name="go2_policy")
    world.reset()
    robot.initialize()

    dof_names = list(robot.dof_names)
    print(f"[policy] DOFs: {dof_names}", flush=True)
    name_to_idx = {n: i for i, n in enumerate(dof_names)}
    missing = [n for n in LAB_JOINT_ORDER if n not in name_to_idx]
    if missing:
        print(f"[FAIL] missing joints {missing}", flush=True)
        return 1
    lab_to_dof = np.array([name_to_idx[n] for n in LAB_JOINT_ORDER], dtype=np.int64)

    controller = robot.get_articulation_controller()
    controller.set_gains(
        kps=np.full(len(dof_names), ARGS.kp, dtype=np.float32),
        kds=np.full(len(dof_names), ARGS.kd, dtype=np.float32),
    )
    robot.set_solver_position_iteration_count(16)
    robot.set_solver_velocity_iteration_count(4)

    # init pose
    q0 = default_pos.copy()
    targets_dof = np.zeros(len(dof_names), dtype=np.float32)
    targets_dof[lab_to_dof] = q0
    zeros = np.zeros(len(dof_names), dtype=np.float32)
    robot.set_joint_positions(targets_dof)
    robot.set_joint_velocities(zeros)
    controller.apply_action(ArticulationAction(joint_positions=targets_dof, joint_velocities=zeros))
    stand_dof = targets_dof.copy()
    spawn_pos, spawn_quat = robot.get_world_pose()
    spawn_pos = np.asarray(spawn_pos, dtype=np.float64).copy()
    spawn_quat = np.asarray(spawn_quat, dtype=np.float64).copy()

    def place_robot(position: np.ndarray, orientation: np.ndarray, label: str) -> None:
        nonlocal targets_dof, last_action
        robot.set_world_pose(position=position, orientation=orientation)
        try:
            robot.set_linear_velocity(np.zeros(3, dtype=np.float64))
            robot.set_angular_velocity(np.zeros(3, dtype=np.float64))
        except Exception as exc:  # noqa: BLE001
            print(f"[{label}] 清速度失败: {exc}", flush=True)
        robot.set_joint_positions(stand_dof)
        robot.set_joint_velocities(zeros)
        controller.apply_action(
            ArticulationAction(joint_positions=stand_dof, joint_velocities=zeros)
        )
        targets_dof = stand_dof.copy()
        last_action = np.zeros(12, dtype=np.float32)
        print(
            f"[{label}] pos=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f})",
            flush=True,
        )

    signal.signal(signal.SIGUSR1, _on_pose_signal)
    signal.signal(signal.SIGUSR2, _on_pose_signal)
    with open(POSE_PIDFILE, "w", encoding="utf-8") as pidf:
        pidf.write(str(os.getpid()))
    print(
        f"[policy] 扶正=SIGUSR2 回原点=SIGUSR1 pid={os.getpid()} "
        f"（{POSE_PIDFILE}）",
        flush=True,
    )

    device = torch.device(ARGS.device if torch.cuda.is_available() else "cpu")
    policy = torch.jit.load(ARGS.policy, map_location=device)
    policy.eval()
    print(f"[policy] loaded {ARGS.policy} on {device}", flush=True)

    cmd = CmdVelRelay(ARGS.cmd_timeout)
    last_action = np.zeros(12, dtype=np.float32)
    decim = max(1, int(round(step_dt * ARGS.physics_hz)))
    print(f"[policy] step_dt={step_dt} physics_hz={ARGS.physics_hz} decimation={decim}", flush=True)
    print("[policy] loop — pub /cmd_vel to walk; Mid360 should publish /livox/*", flush=True)

    step = 0
    try:
        while simulation_app.is_running():
            if _pose_cmd["origin"]:
                _pose_cmd["origin"] = False
                place_robot(spawn_pos, spawn_quat, "origin")
            elif _pose_cmd["standup"]:
                _pose_cmd["standup"] = False
                cur_pos, cur_quat = robot.get_world_pose()
                cur_pos = np.asarray(cur_pos, dtype=np.float64)
                lifted = cur_pos.copy()
                lifted[2] = float(cur_pos[2]) + STANDUP_Z
                place_robot(lifted, _upright_like_spawn(cur_quat, spawn_quat), "standup")
            vx, vy, wz = cmd.read()
            # physics substeps between policy updates
            for _ in range(decim):
                # hold last targets during substeps
                controller.apply_action(ArticulationAction(joint_positions=targets_dof, joint_velocities=zeros))
                # render every policy tick's last phys step so lidar ticks
                world.step(render=(_ == decim - 1))

            pos, quat = robot.get_world_pose()
            # ang vel: Isaac Robot may expose get_angular_velocity in world frame
            try:
                ang_w = np.asarray(robot.get_angular_velocity(), dtype=np.float64)
            except Exception:
                ang_w = np.zeros(3, dtype=np.float64)
            ang_b = world_to_body(ang_w, quat).astype(np.float32)
            grav_w = np.array([0.0, 0.0, -1.0], dtype=np.float64)
            proj_g = world_to_body(grav_w, quat).astype(np.float32)

            q_dof = np.asarray(robot.get_joint_positions(), dtype=np.float32)
            dq_dof = np.asarray(robot.get_joint_velocities(), dtype=np.float32)
            q_lab = q_dof[lab_to_dof]
            dq_lab = dq_dof[lab_to_dof]

            obs = np.concatenate(
                [
                    ang_b * obs_scales["base_ang_vel"],
                    proj_g * obs_scales["projected_gravity"],
                    np.array([vx, vy, wz], dtype=np.float32) * obs_scales["velocity_commands"],
                    (q_lab - default_pos) * obs_scales["joint_pos_rel"],
                    dq_lab * obs_scales["joint_vel_rel"],
                    last_action * obs_scales["last_action"],
                ]
            ).astype(np.float32)

            with torch.inference_mode():
                inp = torch.from_numpy(obs).unsqueeze(0).to(device)
                out = policy(inp)
                if isinstance(out, (tuple, list)):
                    out = out[0]
                act = out.detach().float().cpu().numpy().reshape(-1)[:12]

            last_action = act.astype(np.float32)
            q_des_lab = act_offset + act_scale * last_action
            targets_dof = np.zeros(len(dof_names), dtype=np.float32)
            targets_dof[lab_to_dof] = q_des_lab

            if step % 25 == 0:
                print(
                    f"[cmd_vel] vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}  "
                    f"pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})",
                    flush=True,
                )
            step += 1
    except KeyboardInterrupt:
        print("[policy] interrupted", flush=True)
    finally:
        cmd.close()
        try:
            os.remove(POSE_PIDFILE)
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    finally:
        simulation_app.close()
    sys.exit(code)
