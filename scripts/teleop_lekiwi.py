# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Script to run LeKiwi teleoperation with mobile base + arm control."""

"""Launch Isaac Sim Simulator first."""
import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse

from isaaclab.app import AppLauncher

# Add argparse arguments
parser = argparse.ArgumentParser(description="LeKiwi teleoperation with mobile base + arm control.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port for SO101Leader arm device.")
parser.add_argument("--task", type=str, default="LeKisaac-LeKiwi-Teleop-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed for the environment.")
parser.add_argument("--base_linear_speed", type=float, default=0.3, help="Base linear speed (m/s).")
parser.add_argument("--base_angular_speed", type=float, default=0.5, help="Base angular speed (rad/s).")

# Recorder parameters
parser.add_argument("--record", action="store_true", help="Enable recording.")
parser.add_argument("--step_hz", type=int, default=30, help="Environment stepping rate in Hz.")
parser.add_argument("--dataset_file", type=str, default="./datasets/lekiwi_dataset.hdf5", help="Output file path.")
parser.add_argument("--resume", action="store_true", help="Resume recording in existing dataset file.")
parser.add_argument("--num_demos", type=int, default=0, help="Number of demonstrations to record (0 for infinite).")

parser.add_argument("--recalibrate", action="store_true", help="Recalibrate SO101-Leader arm.")
parser.add_argument("--quality", action="store_true", help="Enable quality render mode.")

# Append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher_args = vars(args_cli)

# Launch omniverse app
app_launcher = AppLauncher(app_launcher_args)
simulation_app = app_launcher.app

import os
import time
import torch
import gymnasium as gym

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.managers import TerminationTermCfg, DatasetExportMode

# Import lekisaac components
import lekisaac  # noqa: F401 - registers gym environments
from lekisaac.devices import LeKiwiDevice
from leisaac.enhance.managers import StreamingRecorderManager, EnhanceDatasetExportMode


class RateLimiter:
    """Convenience class for enforcing rates in loops."""

    def __init__(self, hz):
        self.hz = hz
        self.last_time = time.time()
        self.sleep_duration = 1.0 / hz
        self.render_period = min(0.0166, self.sleep_duration)

    def sleep(self, env):
        """Attempt to sleep at the specified rate in hz."""
        next_wakeup_time = self.last_time + self.sleep_duration
        while time.time() < next_wakeup_time:
            time.sleep(self.render_period)
            env.sim.render()

        self.last_time = self.last_time + self.sleep_duration

        # Detect time jumping forwards (e.g. loop is too slow)
        if self.last_time < time.time():
            while self.last_time < time.time():
                self.last_time += self.sleep_duration


def manual_terminate(env: ManagerBasedRLEnv, success: bool):
    """Manually trigger episode termination."""
    if hasattr(env, "termination_manager"):
        if success:
            env.termination_manager.set_term_cfg(
                "success",
                TerminationTermCfg(
                    func=lambda env: torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
                ),
            )
        else:
            env.termination_manager.set_term_cfg(
                "success",
                TerminationTermCfg(
                    func=lambda env: torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
                ),
            )
        env.termination_manager.compute()


def main():
    """Run LeKiwi teleoperation."""

    # Create output directory if recording
    output_dir = os.path.dirname(args_cli.dataset_file)
    output_file_name = os.path.splitext(os.path.basename(args_cli.dataset_file))[0]
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Parse environment configuration
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device("lekiwi")
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else int(time.time())

    if args_cli.quality:
        env_cfg.sim.render.antialiasing_mode = "FXAA"
        env_cfg.sim.render.rendering_mode = "quality"

    # Disable timeout and success termination for teleoperation
    if hasattr(env_cfg.terminations, "time_out"):
        env_cfg.terminations.time_out = None
    if hasattr(env_cfg.terminations, "success"):
        env_cfg.terminations.success = None

    # Configure recorder if recording
    if args_cli.record:
        if args_cli.resume:
            env_cfg.recorders.dataset_export_mode = EnhanceDatasetExportMode.EXPORT_ALL_RESUME
            assert os.path.exists(args_cli.dataset_file), (
                "Dataset file does not exist. Remove --resume to create a new dataset."
            )
        else:
            env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_ALL
            assert not os.path.exists(args_cli.dataset_file), (
                "Dataset file already exists. Use --resume to continue recording."
            )
        env_cfg.recorders.dataset_export_dir_path = output_dir
        env_cfg.recorders.dataset_filename = output_file_name

        if not hasattr(env_cfg.terminations, "success"):
            env_cfg.terminations.success = None

    # Create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # Replace recorder manager with streaming version if recording
    if args_cli.record:
        env.recorder_manager = StreamingRecorderManager(
            env_cfg.recorders,
            env,
            flush_steps=30,
            episode_stop_condition=args_cli.num_demos if args_cli.num_demos > 0 else None,
        )

    # Create LeKiwi teleoperation device
    teleop_device = LeKiwiDevice(
        env,
        port=args_cli.port,
        recalibrate=args_cli.recalibrate,
        base_linear_speed=args_cli.base_linear_speed,
        base_angular_speed=args_cli.base_angular_speed,
    )

    # Add callbacks for episode termination
    teleop_device.add_callback("R", lambda: manual_terminate(env, success=False))
    teleop_device.add_callback("N", lambda: manual_terminate(env, success=True))

    print(teleop_device)

    # Rate limiter for consistent stepping
    rate_limiter = RateLimiter(args_cli.step_hz)

    # Reset environment
    env.reset()

    # Main teleoperation loop
    print("\n[INFO] Press 'B' to start teleoperation...")

    while simulation_app.is_running():
        # Get action from device
        action = teleop_device.advance()

        if action is None:
            # Not started yet - just render
            env.sim.render()
            continue

        if isinstance(action, dict):
            # Reset requested
            if action.get("reset", False):
                env.reset()
                teleop_device.reset()
                continue

        # Step environment with action
        env.step(action)

        # Enforce stepping rate
        rate_limiter.sleep(env)

        # Check if we should stop (reached demo limit)
        if args_cli.record and args_cli.num_demos > 0:
            if hasattr(env, "recorder_manager"):
                if env.recorder_manager.should_stop:
                    print(f"\n[INFO] Collected {args_cli.num_demos} demonstrations. Stopping.")
                    break

    # Cleanup
    teleop_device.disconnect()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
