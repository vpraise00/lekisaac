# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Observation functions for LeKiwi teleoperation task."""

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def base_linear_velocity(env: "ManagerBasedEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Get the linear velocity of the robot base in local frame.

    Args:
        env: The environment instance.
        asset_cfg: Configuration for the asset.

    Returns:
        Tensor of shape (num_envs, 3) containing base linear velocity [vx, vy, vz].
    """
    asset = env.scene[asset_cfg.name]
    return asset.data.root_lin_vel_b


def base_angular_velocity(env: "ManagerBasedEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Get the angular velocity of the robot base in local frame.

    Args:
        env: The environment instance.
        asset_cfg: Configuration for the asset.

    Returns:
        Tensor of shape (num_envs, 3) containing base angular velocity [wx, wy, wz].
    """
    asset = env.scene[asset_cfg.name]
    return asset.data.root_ang_vel_b


def wheel_velocities(env: "ManagerBasedEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Get the velocities of the wheel joints.

    Args:
        env: The environment instance.
        asset_cfg: Configuration for the asset.

    Returns:
        Tensor of shape (num_envs, 3) containing wheel angular velocities.
    """
    asset = env.scene[asset_cfg.name]
    # Get joint indices for wheels
    wheel_joint_names = ["wheel_left", "wheel_right", "wheel_back"]
    wheel_indices = []
    for name in wheel_joint_names:
        idx = asset.find_joints(name)[0]
        if len(idx) > 0:
            wheel_indices.append(idx[0])

    if len(wheel_indices) == 3:
        return asset.data.joint_vel[:, wheel_indices]
    else:
        # Return zeros if wheel joints not found
        return torch.zeros(env.num_envs, 3, device=env.device)
