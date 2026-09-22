#!/usr/bin/env python3
"""Isaac Lab：Unitree-Go2-Velocity 播放循环 + 订阅 ROS `/cmd_vel`。

在 **env_isaaclab** 中、从 `~/unitree_rl_lab` 调用（与官方 play 相同启动器）：

  conda activate env_isaaclab
  source /opt/ros/humble/setup.bash          # 需要 rclpy
  source ~/navi-sim-code-backups/slam-nav-20260901T135838+0800/setup_ros_local.sh
  cd ~/unitree_rl_lab
  ./unitree_rl_lab.sh -p --task Unitree-Go2-Velocity \\
      --checkpoint <可选> \\
    # 实际请直接：
  python scripts/rsl_rl/play.py ...   # 官方无 cmd_vel
  # 本脚本：
  python /path/to/slam-nav/scripts/lab_play_go2_cmd_vel.py \\
      --task Unitree-Go2-Velocity --viz kit --num_envs 1

把 Nav2 / teleop 的 `/cmd_vel`（机体坐标系：vx, vy, wz）写入
`command_manager["base_velocity"].vel_command_b`，策略观测里的 velocity_commands
与指令一致。

注意：
  - 本阶段仍是 **默认 Lab 平地**，不是大学城 mesh（方案 A 地形替换另做）。
  - 短训权重可能站不稳；先确认 `ros2 topic echo /cmd_vel` 有数据且本脚本日志打印非零指令。
  - 与 Isaac Sim `scene_daxuecheng_go2.usd` 传感器栈并行时，用同一 `ROS_DOMAIN_ID`。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# AppLauncher 必须先于其它 isaac 导入（与 unitree_rl_lab/scripts/rsl_rl/play.py 相同）
# ---------------------------------------------------------------------------
# 允许从任意 cwd 找到 unitree_rl_lab 的 cli_args
_CANDIDATE_ROOTS = [
    os.path.expanduser("~/unitree_rl_lab"),
    os.path.expanduser("~/unitree_rl_lab/scripts/rsl_rl"),
]
for _root in _CANDIDATE_ROOTS:
    _scripts = os.path.join(_root, "scripts", "rsl_rl") if not _root.endswith("rsl_rl") else _root
    if os.path.isdir(_scripts) and _scripts not in sys.path:
        sys.path.insert(0, _scripts)
    _lab = _root if not _root.endswith("rsl_rl") else os.path.dirname(os.path.dirname(_root))
    if os.path.isdir(_lab) and _lab not in sys.path:
        sys.path.insert(0, _lab)

from isaaclab.app import AppLauncher  # noqa: E402

import cli_args  # noqa: E402  # from unitree_rl_lab/scripts/rsl_rl

parser = argparse.ArgumentParser(description="Play Go2 velocity policy driven by ROS /cmd_vel.")
parser.add_argument("--video", action="store_true", default=False)
parser.add_argument("--video_length", type=int, default=200)
parser.add_argument("--disable_fabric", action="store_true", default=False)
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--task", type=str, default="Unitree-Go2-Velocity")
parser.add_argument("--use_pretrained_checkpoint", action="store_true", default=False)
parser.add_argument("--real-time", action="store_true", default=False)
parser.add_argument("--cmd_vel_topic", type=str, default="/cmd_vel")
parser.add_argument("--cmd_timeout", type=float, default=0.5,
                    help="无新 /cmd_vel 超过该秒数则指令清零（安全）")
parser.add_argument(
    "--campus_terrain",
    action="store_true",
    default=False,
    help="阶段 B：用地图大学城 collision USD 替换 Lab generator 地形",
)
parser.add_argument(
    "--campus_usd",
    type=str,
    default="",
    help="大学城 collision USD/USDZ 路径；空则用仓库 assets/daxuecheng/daxuecheng-collision-smooth.usdz",
)
parser.add_argument(
    "--campus_init_xyz",
    type=str,
    default="0,2,-0.014",
    help="校园地形上 spawn 的 x,y,z（默认平坦路段；地面≈-0.41 + 站高0.4）",
)
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.video:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ---------------------------------------------------------------------------
import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab_tasks  # noqa: E402, F401
from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent  # noqa: E402
from isaaclab.utils.assets import retrieve_file_path  # noqa: E402
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)
from isaaclab_tasks.utils import get_checkpoint_path  # noqa: E402
from importlib.metadata import version as pkg_version  # noqa: E402

import unitree_rl_lab.tasks  # noqa: E402, F401
from unitree_rl_lab.utils.parser_cfg import parse_env_cfg  # noqa: E402

try:
    import rclpy
    from geometry_msgs.msg import Twist

    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False


class CmdVelBuffer:
    """订阅 `/cmd_vel`（机体 vx, vy, wz）。优先本进程 rclpy；否则用 /usr/bin/python3 中继。"""

    def __init__(self, topic: str, timeout: float) -> None:
        self.timeout = timeout
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._stamp = 0.0
        self._proc = None
        self.node = None
        if _HAS_RCLPY:
            self.node = rclpy.create_node("lab_go2_cmd_vel")
            self.node.create_subscription(Twist, topic, self._on_cmd, 10)
            self.node.get_logger().info(f"订阅 {topic} → Lab base_velocity（本进程 rclpy）")
        else:
            self._start_relay(topic)

    def _on_cmd(self, msg: Twist) -> None:
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)
        self._stamp = time.time()

    def _start_relay(self, topic: str) -> None:
        """Lab Python 3.12 通常无 Humble rclpy，用系统 3.10 子进程中继。"""
        import subprocess
        import threading

        script_dir = os.path.dirname(os.path.abspath(__file__))
        repo = os.path.dirname(script_dir)
        dds_xml = os.path.join(repo, "cyclonedds_localhost.xml")
        py_relay = os.path.join(script_dir, "_cmd_vel_relay_helper.py")

        bash = f"""
set -e
# 隔离 Isaac/conda 的 PYTHONPATH，避免 Humble 的 /usr/bin/python3 出现 SRE mismatch
env -i \
  HOME="$HOME" USER="$USER" LOGNAME="$LOGNAME" \
  PATH=/usr/bin:/bin \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ROS_DOMAIN_ID=7 \
  ROS_LOCALHOST_ONLY=1 \
  CYCLONEDDS_URI=file://{dds_xml} \
  /bin/bash -lc 'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 {py_relay} {topic}'
"""
        self._proc = subprocess.Popen(
            ["/bin/bash", "-lc", bash],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env={
                "HOME": os.path.expanduser("~"),
                "USER": os.environ.get("USER", ""),
                "LOGNAME": os.environ.get("LOGNAME", ""),
                "PATH": "/usr/bin:/bin",
            },
        )

        def _reader() -> None:
            assert self._proc and self._proc.stdout
            for line in self._proc.stdout:
                line = line.strip()
                if line.startswith("CMD "):
                    parts = line.split()
                    if len(parts) >= 5:
                        self.vx = float(parts[1])
                        self.vy = float(parts[2])
                        self.wz = float(parts[3])
                        self._stamp = float(parts[4])
                elif line:
                    print(f"[cmd_relay] {line}", flush=True)

        threading.Thread(target=_reader, daemon=True).start()
        print(f"[INFO] 无本进程 rclpy，已启动 /usr/bin/python3 中继订阅 {topic}", flush=True)

    def read(self) -> tuple[float, float, float]:
        if time.time() - self._stamp > self.timeout:
            return 0.0, 0.0, 0.0
        return self.vx, self.vy, self.wz

    def spin_once(self) -> None:
        if self.node is not None and _HAS_RCLPY:
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self) -> None:
        if self.node is not None and _HAS_RCLPY:
            self.node.destroy_node()
        if self._proc is not None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except Exception:
                self._proc.kill()


def _apply_campus_terrain(env_cfg) -> str:
    """阶段 B：Lab 地形换成大学城 collision；关闭 terrain curriculum。"""
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd = args_cli.campus_usd.strip() or os.path.join(
        repo, "assets", "daxuecheng", "daxuecheng-collision-smooth.usdz"
    )
    usd = os.path.abspath(os.path.expanduser(usd))
    if not os.path.isfile(usd):
        raise FileNotFoundError(f"campus USD 不存在: {usd}")

    xyz = [float(x) for x in args_cli.campus_init_xyz.split(",")]
    if len(xyz) != 3:
        raise ValueError("--campus_init_xyz 需要 x,y,z 三个数")

    env_cfg.scene.terrain.terrain_type = "usd"
    env_cfg.scene.terrain.usd_path = usd
    env_cfg.scene.terrain.terrain_generator = None
    env_cfg.scene.terrain.max_init_terrain_level = None
    # usd 地形用网格 origins，必须有 env_spacing
    if getattr(env_cfg.scene.terrain, "env_spacing", None) is None:
        env_cfg.scene.terrain.env_spacing = float(getattr(env_cfg.scene, "env_spacing", 5.0) or 5.0)
    env_cfg.curriculum.terrain_levels = None
    env_cfg.scene.robot.init_state.pos = (xyz[0], xyz[1], xyz[2])
    # 缩小 reset 漂移，避免随机扔到建筑上
    env_cfg.events.reset_base.params["pose_range"] = {
        "x": (-0.2, 0.2),
        "y": (-0.2, 0.2),
        "yaw": (-0.3, 0.3),
    }
    print(
        f"[campus] terrain_type=usd usd={usd} init_xyz={tuple(xyz)} "
        f"terrain_levels=None",
        flush=True,
    )
    return usd


def main() -> None:
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
        entry_point_key="play_env_cfg_entry_point",
    )
    # 关闭随机指令 / 站立样本，改由 /cmd_vel 驱动
    env_cfg.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)
    env_cfg.commands.base_velocity.rel_standing_envs = 0.0
    if args_cli.campus_terrain:
        _apply_campus_terrain(env_cfg)

    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)
    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", args_cli.task)
        if not resume_path:
            print("[INFO] No pretrained checkpoint for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, pkg_version("rsl-rl-lib"))

    print(f"[INFO] Loading checkpoint: {resume_path}")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    if _HAS_RCLPY:
        rclpy.init(args=None)
    cmd = CmdVelBuffer(args_cli.cmd_vel_topic, args_cli.cmd_timeout)
    dt = env.unwrapped.step_dt
    obs = env.get_observations()
    step_i = 0

    print("[INFO] play_cmd_vel loop — teleop: ros2 topic pub /cmd_vel geometry_msgs/Twist ...")
    while simulation_app.is_running():
        t0 = time.time()
        cmd.spin_once()
        vx, vy, wz = cmd.read()
        term = env.unwrapped.command_manager.get_term("base_velocity")
        term.vel_command_b[:, 0] = vx
        term.vel_command_b[:, 1] = vy
        term.vel_command_b[:, 2] = wz
        if hasattr(term, "is_standing_env"):
            term.is_standing_env[:] = False

        # 必须用 wrapper.get_observations()（TensorDict），勿直接 observation_manager.compute()
        # 否则 rsl-rl 5 MLPModel 会 IndexError: too many indices for tensor of dimension 2
        with torch.inference_mode():
            obs = env.get_observations()
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

        if step_i % 50 == 0:
            try:
                robot = env.unwrapped.scene["robot"]
                root = robot.data.root_pos_w[0].detach().cpu().numpy()
                ang = robot.data.root_ang_vel_b[0].detach().cpu().numpy()
                # 机身抖动代理：过大则行走时雷达易叠影 / 平地误判
                ang_xy = float((ang[0] ** 2 + ang[1] ** 2) ** 0.5)
                extra = f" ang_xy={ang_xy:.3f}"
                try:
                    hs = env.unwrapped.scene["height_scanner"]
                    # RayCaster 高度相对值；方差大 = 地面「台阶」假象风险
                    h = hs.data.ray_hits_w[..., 2]
                    if h is not None and h.numel() > 0:
                        hv = h[0].detach().cpu().numpy()
                        hv = hv[np.isfinite(hv)]
                        if hv.size:
                            extra += f" hscan_std={float(hv.std()):.3f}"
                except Exception:
                    pass
                print(
                    f"[cmd_vel] vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}  "
                    f"pos=({root[0]:.2f},{root[1]:.2f},{root[2]:.2f}){extra}",
                    flush=True,
                )
            except Exception:
                print(f"[cmd_vel] vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}", flush=True)
        step_i += 1

        sleep_t = dt - (time.time() - t0)
        if args_cli.real_time and sleep_t > 0:
            time.sleep(sleep_t)

    cmd.shutdown()
    if _HAS_RCLPY:
        rclpy.shutdown()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
