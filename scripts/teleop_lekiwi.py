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
    """Manually trigger episode termination and mark success/failure."""
    if hasattr(env, "termination_manager"):
        # Try to set success termination term
        try:
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
        except ValueError:
            # Term doesn't exist, just store in extras
            pass

    # Store success flag in extras for recorder to capture
    env.extras["episode_success"] = success


def main():
    """Run LeKiwi teleoperation."""

    # Create output directory if recording
    output_dir = os.path.dirname(args_cli.dataset_file)
    output_file_name = os.path.splitext(os.path.basename(args_cli.dataset_file))[0]
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Parse environment configuration
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device("lekiwi")
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else int(time.time())

    # Enable manual termination mode
    if hasattr(env_cfg, "manual_terminate"):
        env_cfg.manual_terminate = True

    if args_cli.quality:
        env_cfg.sim.render.antialiasing_mode = "FXAA"
        env_cfg.sim.render.rendering_mode = "quality"

    # Disable timeout and success termination for teleoperation
    if hasattr(env_cfg.terminations, "time_out"):
        env_cfg.terminations.time_out = None
    if hasattr(env_cfg.terminations, "success"):
        env_cfg.terminations.success = None

    # Configure recorder
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
    else:
        # Disable recording completely to prevent memory accumulation
        env_cfg.recorders = None

    # Create environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    # Get unwrapped environment for direct access to IsaacLab internals
    unwrapped_env = env.unwrapped

    # Replace recorder manager with streaming version if recording
    if args_cli.record:
        unwrapped_env.recorder_manager = StreamingRecorderManager(
            env_cfg.recorders,
            unwrapped_env,
            flush_steps=30,
            episode_stop_condition=args_cli.num_demos if args_cli.num_demos > 0 else None,
        )

    # Create LeKiwi teleoperation device
    teleop_device = LeKiwiDevice(
        unwrapped_env,
        port=args_cli.port,
        recalibrate=args_cli.recalibrate,
        base_linear_speed=args_cli.base_linear_speed,
        base_angular_speed=args_cli.base_angular_speed,
    )

    # Flags for handling reset in main loop (leisaac pattern)
    should_reset_recording_instance = False
    should_reset_task_success = False

    def reset_recording_instance():
        nonlocal should_reset_recording_instance
        should_reset_recording_instance = True

    def reset_task_success():
        nonlocal should_reset_task_success
        should_reset_task_success = True
        reset_recording_instance()  # Also trigger reset

    # Add callbacks - just set flags, handle in main loop
    teleop_device.add_callback("R", reset_recording_instance)
    teleop_device.add_callback("N", reset_task_success)

    # Rate limiter for consistent stepping
    rate_limiter = RateLimiter(args_cli.step_hz)

    # Reset environment
    env.reset()
    teleop_device.reset()

    # Track recording state and demo count
    resume_recorded_demo_count = 0
    if args_cli.record and args_cli.resume:
        resume_recorded_demo_count = unwrapped_env.recorder_manager._dataset_file_handler.get_num_episodes()
        print(f"[INFO] Resume recording from existing dataset file with {resume_recorded_demo_count} demonstrations.")
    current_recorded_demo_count = resume_recorded_demo_count
    start_record_state = False

    # Print device controls after environment initialization (so it appears after PhysX warnings)
    print(teleop_device)
    print("[INFO] Press 'B' to start teleoperation...")

    while simulation_app.is_running():
        with torch.inference_mode():
            # Get action from device
            actions = teleop_device.advance()

            # Handle task success (N key pressed)
            if should_reset_task_success:
                print("[INFO] Task Success!")
                should_reset_task_success = False
                if args_cli.record:
                    manual_terminate(unwrapped_env, True)

            # Handle reset (R key pressed, or after N key)
            if should_reset_recording_instance:
                env.reset()
                teleop_device.reset()
                should_reset_recording_instance = False

                if start_record_state:
                    if args_cli.record:
                        print("[INFO] Stop Recording!")
                    start_record_state = False

                if args_cli.record:
                    manual_terminate(unwrapped_env, False)

                # Print demo count if it changed
                if args_cli.record:
                    new_count = unwrapped_env.recorder_manager.exported_successful_episode_count + resume_recorded_demo_count
                    if new_count > current_recorded_demo_count:
                        current_recorded_demo_count = new_count
                        print(f"[INFO] Recorded {current_recorded_demo_count} successful demonstrations.")

                    # Check if we reached the target number of demos
                    if args_cli.num_demos > 0 and new_count >= args_cli.num_demos:
                        print(f"[INFO] All {args_cli.num_demos} demonstrations recorded. Exiting.")
                        break

            elif actions is None or isinstance(actions, dict):
                # Not started yet or reset dict returned - just render
                unwrapped_env.sim.render()
                continue  # Skip rate limiter when not stepping

            else:
                # Started - step environment with tensor action
                if not start_record_state:
                    if args_cli.record:
                        print("[INFO] Start Recording!")
                    start_record_state = True
                env.step(actions)
                # Enforce stepping rate only when stepping
                rate_limiter.sleep(unwrapped_env)

    # Cleanup
    teleop_device.disconnect()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
