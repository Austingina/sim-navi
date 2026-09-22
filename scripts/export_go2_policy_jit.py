#!/usr/bin/env python3
"""Export Unitree-Go2-Velocity policy to TorchScript for Isaac standalone play (阶段 C).

在 env_isaaclab + ~/unitree_rl_lab 下运行：

  python scripts/export_go2_policy_jit.py \\
    --checkpoint logs/rsl_rl/unitree_go2_velocity/2026-09-22_11-04-32/model_11097.pt
"""
from __future__ import annotations

import argparse
import os
import sys

_ROOTS = [
    os.path.expanduser("~/unitree_rl_lab"),
    os.path.expanduser("~/unitree_rl_lab/scripts/rsl_rl"),
]
for _root in _ROOTS:
    _scripts = os.path.join(_root, "scripts", "rsl_rl") if not _root.endswith("rsl_rl") else _root
    if os.path.isdir(_scripts) and _scripts not in sys.path:
        sys.path.insert(0, _scripts)
    _lab = _root if not _root.endswith("rsl_rl") else os.path.dirname(os.path.dirname(_root))
    if os.path.isdir(_lab) and _lab not in sys.path:
        sys.path.insert(0, _lab)

from isaaclab.app import AppLauncher  # noqa: E402
import cli_args  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Unitree-Go2-Velocity")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--out_dir", default="", help="默认 checkpoint 同级 exported/")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if not getattr(args_cli, "checkpoint", None):
    raise SystemExit("需要 --checkpoint <model_XXXX.pt>")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from importlib.metadata import version as pkg_version  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import unitree_rl_lab.tasks  # noqa: E402, F401
from isaaclab.utils.assets import retrieve_file_path  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)
from unitree_rl_lab.utils.parser_cfg import parse_env_cfg  # noqa: E402


def main() -> None:
    ckpt = retrieve_file_path(args_cli.checkpoint)
    out_dir = args_cli.out_dir.strip() or os.path.join(os.path.dirname(ckpt), "exported")
    os.makedirs(out_dir, exist_ok=True)

    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        entry_point_key="play_env_cfg_entry_point",
    )
    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, pkg_version("rsl-rl-lib"))

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(ckpt)
    print(f"[export] loaded {ckpt}", flush=True)

    jit_path = os.path.join(out_dir, "policy.pt")
    onnx_path = os.path.join(out_dir, "policy.onnx")
    if hasattr(runner, "export_policy_to_jit"):
        runner.export_policy_to_jit(path=out_dir, filename="policy.pt")
        print(f"[export] JIT → {jit_path}", flush=True)
    if hasattr(runner, "export_policy_to_onnx"):
        runner.export_policy_to_onnx(path=out_dir, filename="policy.onnx")
        print(f"[export] ONNX → {onnx_path}", flush=True)

    # Always also dump a traced module from get_inference_policy (obs TensorDict → action)
    policy = runner.get_inference_policy(device="cpu")
    obs = env.get_observations()
    # Flatten like deploy expects: single tensor [1, obs_dim]
    if hasattr(obs, "get"):
        # TensorDict
        try:
            flat = obs["policy"]
        except Exception:
            flat = obs
    else:
        flat = obs
    if isinstance(flat, dict):
        flat = next(iter(flat.values()))
    flat = flat.detach().to("cpu")
    if flat.dim() == 1:
        flat = flat.unsqueeze(0)

    class _PolicyWrap(torch.nn.Module):
        def __init__(self, fn):
            super().__init__()
            self.fn = fn

        def forward(self, x: torch.Tensor) -> torch.Tensor:
            return self.fn(x)

    # rsl-rl 5 may expect TensorDict; try tensor path first
    try:
        with torch.inference_mode():
            act = policy(flat)
        print(f"[export] tensor forward ok action={tuple(act.shape)}", flush=True)
        wrapped = _PolicyWrap(policy)
        traced = torch.jit.trace(wrapped, flat, strict=False)
        trace_path = os.path.join(out_dir, "policy_traced.pt")
        traced.save(trace_path)
        print(f"[export] traced → {trace_path}", flush=True)
    except Exception as e:
        print(f"[export] tensor/trace skip: {e}", flush=True)
        with torch.inference_mode():
            act = policy(obs)
        print(f"[export] TensorDict forward ok type={type(act)}", flush=True)

    print(f"[export] done out_dir={out_dir}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
