# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Custom event functions for LeKiwi cut ball task."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def check_and_cut_cube(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    knife_cfg: SceneEntityCfg,
    cube_cfg: SceneEntityCfg,
    cube_half_left_cfg: SceneEntityCfg,
    cube_half_right_cfg: SceneEntityCfg,
    cut_threshold: float,
):
    """Check if knife has cut the cube and split it into halves.

    When the knife's z position is close enough to the cube's z position (pressing down),
    the cube is "cut" - original cube is hidden and two half cubes appear.

    Args:
        env: The environment instance.
        knife_cfg: Configuration for the knife asset.
        cube_cfg: Configuration for the cube asset.
        cube_half_left_cfg: Configuration for the left half cube.
        cube_half_right_cfg: Configuration for the right half cube.
        cut_threshold: Z distance threshold for cutting (meters).
    """
    # Get assets
    knife = env.scene[knife_cfg.name]
    cube = env.scene[cube_cfg.name]
    cube_half_left = env.scene[cube_half_left_cfg.name]
    cube_half_right = env.scene[cube_half_right_cfg.name]

    # Get positions
    knife_pos = knife.data.root_pos_w  # (num_envs, 3)
    cube_pos = cube.data.root_pos_w  # (num_envs, 3)

    # Check if knife is directly above cube and pressing down into it
    # Knife must be very close in x-y (within 3cm) and pressed down into cube
    xy_distance = torch.sqrt((knife_pos[:, 0] - cube_pos[:, 0])**2 + (knife_pos[:, 1] - cube_pos[:, 1])**2)
    z_diff = knife_pos[:, 2] - cube_pos[:, 2]  # Positive if knife above cube

    # Cutting condition: knife directly above (within 3cm x-y) and pressed into cube (z below threshold)
    cut_mask = (xy_distance < 0.03) & (z_diff < cut_threshold) & (z_diff > -0.03)

    # Check if cube is still in scene (not already cut - z > -5)
    cube_active = cube_pos[:, 2] > -5.0

    # Combined mask: should cut and cube is active
    should_cut = cut_mask & cube_active

    if should_cut.any():
        # Get indices of environments where cutting happens
        cut_indices = should_cut.nonzero(as_tuple=False).squeeze(-1)

        # Store original cube positions before hiding
        original_cube_pos = cube_pos[cut_indices].clone()

        # Hide original cube (move far below)
        new_cube_pos = cube_pos.clone()
        new_cube_pos[cut_indices, 2] = -10.0  # Move far below
        cube.write_root_pose_to_sim(
            torch.cat([new_cube_pos, cube.data.root_quat_w], dim=-1)
        )

        # Place left half where cube was (offset in X for vertical split)
        left_pos = cube_half_left.data.root_pos_w.clone()
        left_pos[cut_indices, 0] = original_cube_pos[:, 0] - 0.025  # Offset back (negative X)
        left_pos[cut_indices, 1] = original_cube_pos[:, 1]
        left_pos[cut_indices, 2] = original_cube_pos[:, 2]
        cube_half_left.write_root_pose_to_sim(
            torch.cat([left_pos, cube_half_left.data.root_quat_w], dim=-1)
        )

        # Place right half where cube was (offset in X for vertical split)
        right_pos = cube_half_right.data.root_pos_w.clone()
        right_pos[cut_indices, 0] = original_cube_pos[:, 0] + 0.025  # Offset front (positive X)
        right_pos[cut_indices, 1] = original_cube_pos[:, 1]
        right_pos[cut_indices, 2] = original_cube_pos[:, 2]
        cube_half_right.write_root_pose_to_sim(
            torch.cat([right_pos, cube_half_right.data.root_quat_w], dim=-1)
        )
