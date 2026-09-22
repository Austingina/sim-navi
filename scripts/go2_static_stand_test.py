#!/usr/bin/env python3
"""Bounded static-PD standing test for the Go2 in the university USD stage."""
from __future__ import annotations

import argparse
import math
import os
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, help="Absolute path to scene_daxuecheng_go2.usd")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulated test duration [s]")
    parser.add_argument("--physics-hz", type=float, default=200.0, help="Physics frequency [Hz]")
    parser.add_argument("--kp", type=float, default=25.0, help="Joint position stiffness")
    parser.add_argument("--kd", type=float, default=0.5, help="Joint velocity damping")
    parser.add_argument(
        "--spawn-height",
        type=float,
        default=None,
        help="Optional base world-frame height override [m]; default preserves the stage pose",
    )
    parser.add_argument(
        "--min-clearance",
        type=float,
        default=0.18,
        help="Minimum final-window base-to-lowest-foot vertical clearance [m]",
    )
    parser.add_argument(
        "--max-height-span",
        type=float,
        default=0.05,
        help="Maximum base-height range during the final window [m]",
    )
    parser.add_argument("--max-tilt-deg", type=float, default=35.0, help="Maximum final-window roll/pitch [deg]")
    return parser.parse_args()


ARGS = parse_args()
if not os.path.isabs(ARGS.usd) or not os.path.isfile(ARGS.usd):
    raise SystemExit(f"[FAIL] --usd must be an existing absolute file: {ARGS.usd}")
if ARGS.duration <= 0 or ARGS.physics_hz <= 0:
    raise SystemExit("[FAIL] --duration and --physics-hz must be positive")

# SimulationApp must be created before importing omni/Isaac modules.
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True, "disable_viewport_updates": True})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

ROBOT_PRIM = "/World/go2/base"
EXPECTED_JOINTS = {
    "FR_hip_joint": -0.1,
    "FR_thigh_joint": 0.8,
    "FR_calf_joint": -1.5,
    "FL_hip_joint": 0.1,
    "FL_thigh_joint": 0.8,
    "FL_calf_joint": -1.5,
    "RR_hip_joint": -0.1,
    "RR_thigh_joint": 1.0,
    "RR_calf_joint": -1.5,
    "RL_hip_joint": 0.1,
    "RL_thigh_joint": 1.0,
    "RL_calf_joint": -1.5,
}


def quat_to_roll_pitch_deg(quat_wxyz: np.ndarray) -> tuple[float, float]:
    """Convert a scalar-first quaternion to roll and pitch in degrees."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def fail(message: str) -> int:
    print(f"[FAIL] {message}", flush=True)
    return 1


def main() -> int:
    print(f"[stand] opening {ARGS.usd}", flush=True)
    open_stage(ARGS.usd)
    while is_stage_loading():
        simulation_app.update()

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / ARGS.physics_hz,
        rendering_dt=1.0 / ARGS.physics_hz,
    )
    robot = Robot(prim_path=ROBOT_PRIM, name="go2_static_stand")

    # reset() starts physics and gives the articulation a valid tensor handle.
    world.reset()
    robot.initialize()

    dof_names = list(robot.dof_names)
    print(f"[stand] DOFs ({len(dof_names)}): {dof_names}", flush=True)
    missing = sorted(set(EXPECTED_JOINTS) - set(dof_names))
    unexpected = sorted(set(dof_names) - set(EXPECTED_JOINTS))
    if missing or unexpected or len(dof_names) != 12:
        return fail(f"joint contract mismatch; missing={missing}, unexpected={unexpected}")

    body_names = list(robot._articulation_view.body_names)  # noqa: SLF001
    foot_indices = [i for i, name in enumerate(body_names) if name.endswith("_foot")]
    print(f"[stand] bodies ({len(body_names)}): {body_names}", flush=True)
    if len(foot_indices) != 4:
        return fail(f"expected four *_foot links, got indices={foot_indices}")

    targets = np.array([EXPECTED_JOINTS[name] for name in dof_names], dtype=np.float32)
    zeros = np.zeros(len(dof_names), dtype=np.float32)
    controller = robot.get_articulation_controller()
    controller.set_gains(
        kps=np.full(len(dof_names), ARGS.kp, dtype=np.float32),
        kds=np.full(len(dof_names), ARGS.kd, dtype=np.float32),
    )
    robot.set_solver_position_iteration_count(16)
    robot.set_solver_velocity_iteration_count(4)

    authored_position, authored_orientation = robot.get_world_pose()
    print(
        f"[stand] authored/reset base pose: xyz={np.asarray(authored_position).tolist()} "
        f"quat_wxyz={np.asarray(authored_orientation).tolist()}",
        flush=True,
    )
    if ARGS.spawn_height is not None:
        spawn_position = np.asarray(authored_position, dtype=np.float32).copy()
        spawn_position[2] = ARGS.spawn_height
        robot.set_world_pose(position=spawn_position, orientation=authored_orientation)
        print(f"[stand] applied base height override: z={ARGS.spawn_height:.4f}m", flush=True)
    robot.set_joint_positions(targets)
    robot.set_joint_velocities(zeros)
    controller.apply_action(ArticulationAction(joint_positions=targets, joint_velocities=zeros))

    position0, orientation0 = robot.get_world_pose()
    roll0, pitch0 = quat_to_roll_pitch_deg(orientation0)
    print(
        f"[sample] t=0.000 z={position0[2]:.4f} roll={roll0:.2f} pitch={pitch0:.2f} "
        f"joint_max_err={np.max(np.abs(robot.get_joint_positions() - targets)):.4f}",
        flush=True,
    )

    total_steps = max(1, int(round(ARGS.duration * ARGS.physics_hz)))
    final_window_steps = max(1, int(round(min(2.0, ARGS.duration) * ARGS.physics_hz)))
    sample_every = max(1, int(round(ARGS.physics_hz)))
    final_heights: list[float] = []
    final_clearances: list[float] = []
    final_tilts: list[float] = []
    max_joint_error = 0.0
    start_xy = np.asarray(position0[:2], dtype=np.float64)
    end_position = np.asarray(position0, dtype=np.float64)
    end_roll = roll0
    end_pitch = pitch0

    for step in range(1, total_steps + 1):
        # Re-issue the zero-command standing target every physics step.
        controller.apply_action(ArticulationAction(joint_positions=targets, joint_velocities=zeros))
        world.step(render=False)

        joint_positions = np.asarray(robot.get_joint_positions(), dtype=np.float64)
        if not np.all(np.isfinite(joint_positions)):
            return fail(f"non-finite joint state at step {step}")
        joint_error = float(np.max(np.abs(joint_positions - targets)))
        max_joint_error = max(max_joint_error, joint_error)

        position, orientation = robot.get_world_pose()
        end_position = np.asarray(position, dtype=np.float64)
        end_roll, end_pitch = quat_to_roll_pitch_deg(np.asarray(orientation))
        if not np.all(np.isfinite(end_position)) or not math.isfinite(end_roll + end_pitch):
            return fail(f"non-finite base pose at step {step}")

        if step > total_steps - final_window_steps:
            final_heights.append(float(end_position[2]))
            link_transforms = np.asarray(
                robot._articulation_view._physics_view.get_link_transforms(),  # noqa: SLF001
                dtype=np.float64,
            )
            foot_z = link_transforms[0, foot_indices, 2]
            final_clearances.append(float(end_position[2] - np.min(foot_z)))
            final_tilts.append(max(abs(end_roll), abs(end_pitch)))

        if step % sample_every == 0 or step == total_steps:
            print(
                f"[sample] t={step / ARGS.physics_hz:.3f} z={end_position[2]:.4f} "
                f"roll={end_roll:.2f} pitch={end_pitch:.2f} joint_max_err={joint_error:.4f}",
                flush=True,
            )

    min_final_height = min(final_heights)
    final_height_span = max(final_heights) - min(final_heights)
    min_final_clearance = min(final_clearances)
    max_final_tilt = max(final_tilts)
    xy_drift = float(np.linalg.norm(end_position[:2] - start_xy))
    print(
        f"[result] duration={ARGS.duration:.3f}s min_final_height={min_final_height:.4f}m "
        f"height_span={final_height_span:.4f}m min_base_foot_clearance={min_final_clearance:.4f}m "
        f"max_final_tilt={max_final_tilt:.2f}deg xy_drift={xy_drift:.4f}m "
        f"max_joint_error={max_joint_error:.4f}rad",
        flush=True,
    )

    if min_final_clearance < ARGS.min_clearance:
        return fail(
            f"base-foot clearance {min_final_clearance:.4f}m < {ARGS.min_clearance:.4f}m"
        )
    if final_height_span > ARGS.max_height_span:
        return fail(
            f"final base-height span {final_height_span:.4f}m > {ARGS.max_height_span:.4f}m"
        )
    if max_final_tilt > ARGS.max_tilt_deg:
        return fail(f"base tilt {max_final_tilt:.2f}deg > {ARGS.max_tilt_deg:.2f}deg")

    print("[PASS] Go2 held the nominal standing pose under continuous static PD control", flush=True)
    return 0


try:
    EXIT_CODE = main()
except Exception as exc:  # noqa: BLE001
    import traceback

    traceback.print_exc()
    EXIT_CODE = fail(f"unhandled exception: {exc}")
finally:
    simulation_app.close()

sys.exit(EXIT_CODE)
