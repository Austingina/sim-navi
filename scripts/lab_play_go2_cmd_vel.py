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
except ImportError as e:
    raise SystemExit(
        "需要 rclpy。请先 source /opt/ros/humble/setup.bash 再用 Isaac Lab 的 python 跑本脚本。"
    ) from e


class CmdVelBuffer:
    """线程外：spin_once 更新；策略循环读取。"""

    def __init__(self, topic: str, timeout: float) -> None:
        self.timeout = timeout
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._stamp = 0.0
        self.node = rclpy.create_node("lab_go2_cmd_vel")
        self.node.create_subscription(Twist, topic, self._on_cmd, 10)
        self.node.get_logger().info(f"订阅 {topic} → Lab base_velocity 指令")

    def _on_cmd(self, msg: Twist) -> None:
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)
        self._stamp = time.time()

    def read(self) -> tuple[float, float, float]:
        if time.time() - self._stamp > self.timeout:
            return 0.0, 0.0, 0.0
        return self.vx, self.vy, self.wz

    def spin_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)


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
        # 站立掩码清零（若实现存在）
        if hasattr(term, "is_standing_env"):
            term.is_standing_env[:] = False

        # 用最新指令重算观测，再推理
        raw = env.unwrapped.observation_manager.compute()
        if isinstance(raw, dict):
            obs = raw["policy"] if "policy" in raw else next(iter(raw.values()))
        else:
            obs = raw

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

        if step_i % 50 == 0:
            print(f"[cmd_vel] vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
        step_i += 1

        sleep_t = dt - (time.time() - t0)
        if args_cli.real_time and sleep_t > 0:
            time.sleep(sleep_t)

    cmd.node.destroy_node()
    rclpy.shutdown()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
