# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Action processing for LeKiwi robot teleoperation."""

import torch
from typing import Any

import isaaclab.envs.mdp as mdp

from leisaac.assets.robots.lerobot import SO101_FOLLOWER_USD_JOINT_LIMLITS


def init_lekiwi_action_cfg(action_cfg, device):
    """Initialize action configuration for LeKiwi device.

    Args:
        action_cfg: The action configuration object to modify.
        device: The teleoperation device type string.

    Returns:
        The modified action configuration.
    """
    if device == "lekiwi":
        # Arm actions (joint position control)
        action_cfg.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            scale=1.0,
        )
        # Gripper action (joint position control)
        action_cfg.gripper_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["gripper"],
            scale=1.0,
        )
        # Base velocity action (for omnidirectional movement)
        action_cfg.base_velocity_action = mdp.JointVelocityActionCfg(
            asset_name="robot",
            joint_names=["wheel_left", "wheel_right", "wheel_back"],
            scale=1.0,
        )
    else:
        action_cfg.arm_action = None
        action_cfg.gripper_action = None
        action_cfg.base_velocity_action = None
    return action_cfg


# Joint names to motor IDs mapping (same as SO101)
joint_names_to_motor_ids = {
    "shoulder_pan": 0,
    "shoulder_lift": 1,
    "elbow_flex": 2,
    "wrist_flex": 3,
    "wrist_roll": 4,
    "gripper": 5,
}


def convert_arm_action_from_so101_leader(
    joint_state: dict[str, float],
    motor_limits: dict[str, tuple[float, float]],
    teleop_device,
) -> torch.Tensor:
    """Convert SO101Leader joint state to arm action tensor.

    Args:
        joint_state: Dictionary of joint positions from SO101Leader.
        motor_limits: Dictionary of motor position limits.
        teleop_device: The teleoperation device instance.

    Returns:
        Tensor of shape (num_envs, 6) containing arm joint positions in radians.
    """
    processed_action = torch.zeros(teleop_device.env.num_envs, 6, device=teleop_device.env.device)
    joint_limits = SO101_FOLLOWER_USD_JOINT_LIMLITS

    for joint_name, motor_id in joint_names_to_motor_ids.items():
        motor_limit_range = motor_limits[joint_name]
        joint_limit_range = joint_limits[joint_name]

        # Map motor position to joint angle in degrees
        processed_degree = (
            (joint_state[joint_name] - motor_limit_range[0])
            / (motor_limit_range[1] - motor_limit_range[0])
            * (joint_limit_range[1] - joint_limit_range[0])
            + joint_limit_range[0]
        )
        # Convert degrees to radians
        processed_radius = processed_degree / 180.0 * torch.pi
        processed_action[:, motor_id] = processed_radius

    return processed_action


def convert_base_velocity_to_wheel_velocities(
    base_velocity: torch.Tensor,
    wheel_radius: float = 0.048,  # 4" omni wheel radius in meters
    base_radius: float = 0.1,     # Distance from base center to wheel
) -> torch.Tensor:
    """Convert base velocity command to individual wheel velocities.

    Uses inverse kinematics for a 3-wheel omnidirectional robot with
    wheels arranged in a Y configuration (120 degrees apart).

    Args:
        base_velocity: Tensor of shape (num_envs, 3) with [linear_x, linear_y, angular_z].
        wheel_radius: Radius of each wheel in meters.
        base_radius: Distance from base center to wheel axis.

    Returns:
        Tensor of shape (num_envs, 3) with wheel angular velocities [left, right, back].
    """
    # Extract velocity components
    vx = base_velocity[:, 0]  # Forward velocity
    vy = base_velocity[:, 1]  # Left strafe velocity
    wz = base_velocity[:, 2]  # Counter-clockwise angular velocity

    # Wheel positions for Y-configuration (120 degrees apart)
    # Front-left wheel at 120 degrees
    # Front-right wheel at 60 degrees
    # Back wheel at 270 degrees (pointing backward)

    # Inverse kinematics matrix for 3-wheel omni robot
    # Each row represents one wheel's contribution
    # [cos(theta), sin(theta), base_radius] for each wheel

    import math

    # Wheel angles from base center (Y-configuration)
    theta_left = math.radians(120)   # Front-left
    theta_right = math.radians(60)   # Front-right
    theta_back = math.radians(270)   # Back

    # Calculate wheel linear velocities at contact point
    # v_wheel = vx * sin(theta) - vy * cos(theta) + wz * base_radius
    v_left = vx * math.sin(theta_left) - vy * math.cos(theta_left) + wz * base_radius
    v_right = vx * math.sin(theta_right) - vy * math.cos(theta_right) + wz * base_radius
    v_back = vx * math.sin(theta_back) - vy * math.cos(theta_back) + wz * base_radius

    # Convert linear velocity to angular velocity: omega = v / r
    omega_left = v_left / wheel_radius
    omega_right = v_right / wheel_radius
    omega_back = v_back / wheel_radius

    # Stack into output tensor
    wheel_velocities = torch.stack([omega_left, omega_right, omega_back], dim=-1)

    return wheel_velocities


def preprocess_lekiwi_device_action(action: dict[str, Any], teleop_device) -> torch.Tensor:
    """Process LeKiwi device action into environment action tensor.

    Args:
        action: Dictionary containing device action data.
        teleop_device: The teleoperation device instance.

    Returns:
        Tensor containing the processed action for the environment.
        Shape: (num_envs, 9) = [arm(5) + gripper(1) + base(3)]
    """
    if action.get("lekiwi_device") is not None:
        # Process arm action from SO101Leader
        arm_action = convert_arm_action_from_so101_leader(
            action["joint_state"],
            action["motor_limits"],
            teleop_device,
        )

        # Process base velocity from keyboard
        base_velocity = torch.tensor(
            action["base_velocity"],
            device=teleop_device.env.device,
            dtype=torch.float32,
        )
        base_velocity = base_velocity.unsqueeze(0).expand(teleop_device.env.num_envs, -1)

        # Convert base velocity to wheel velocities
        wheel_velocities = convert_base_velocity_to_wheel_velocities(base_velocity)

        # Combine arm (6 DOF) + wheel (3 DOF) actions
        # arm_action[:, :5] = arm joints, arm_action[:, 5] = gripper
        processed_action = torch.cat([arm_action, wheel_velocities], dim=-1)

        return processed_action
    else:
        raise NotImplementedError("Only lekiwi_device teleoperation is supported.")
