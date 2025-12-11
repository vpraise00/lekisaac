# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Holonomic base velocity action term for LeKiwi robot.

This action term directly sets the robot's root body velocity instead of
relying on wheel physics simulation. This bypasses PhysX's lack of anisotropic
friction support, which causes wobbling when using standard wheel velocity control.

Wheel animations are computed using the existing convert_base_velocity_to_wheel_velocities()
logic for visual consistency, but don't affect physics (ground friction = 0).
"""

from __future__ import annotations

import math
import torch
from dataclasses import MISSING
from typing import TYPE_CHECKING

from isaaclab.envs.mdp.actions import ActionTerm, ActionTermCfg
from isaaclab.utils.configclass import configclass
from isaaclab.utils.math import quat_apply_inverse

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class HolonomicBaseVelocityAction(ActionTerm):
    """Action term that directly sets root velocity for holonomic base movement.

    This action bypasses wheel physics by directly setting the robot's root body
    velocity via write_root_velocity_to_sim(). Ground friction is set to 0, so
    wheel rotation is purely visual animation.
    """

    cfg: "HolonomicBaseVelocityActionCfg"

    def __init__(self, cfg: "HolonomicBaseVelocityActionCfg", env: "ManagerBasedEnv"):
        """Initialize the action term."""
        super().__init__(cfg, env)

        # Resolve wheel joint indices for visual animation
        self._wheel_joint_ids, _ = self._asset.find_joints(self.cfg.wheel_joint_names)
        if len(self._wheel_joint_ids) != 3:
            raise ValueError(
                f"Expected 3 wheel joints, got {len(self._wheel_joint_ids)}. "
                f"Joint names: {self.cfg.wheel_joint_names}"
            )

        # Store parameters
        self._wheel_radius = self.cfg.wheel_radius
        self._base_radius = self.cfg.base_radius

        # Pre-allocate tensors
        self._root_velocity = torch.zeros(self.num_envs, 6, device=self.device)
        self._wheel_velocities = torch.zeros(self.num_envs, 3, device=self.device)

    @property
    def action_dim(self) -> int:
        """Dimension of the action space: [vx, vy, wz]."""
        return 3

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        """Process input actions (already scaled by keyboard input)."""
        self._raw_actions = actions.clone()
        self._processed_actions = actions.clone()

    def apply_actions(self):
        """Apply the processed actions to the simulation."""
        # Get robot orientation (world frame)
        root_quat = self._asset.data.root_quat_w

        # Get velocity commands [vx, vy, wz] from user input
        vx_user = self._processed_actions[:, 0]  # User: W/S = forward/backward
        vy_user = self._processed_actions[:, 1]  # User: A/D = left/right
        wz = self._processed_actions[:, 2]  # CCW rotation

        # Transform from user frame to robot URDF frame
        # Robot's "forward" (arm direction) is -Y in URDF
        # User vx (forward) -> Robot +Y
        # User vy (left) -> Robot -X
        vx_robot = -vy_user
        vy_robot = vx_user

        # Convert local velocity to world frame
        local_lin_vel = torch.zeros(self.num_envs, 3, device=self.device)
        local_lin_vel[:, 0] = vx_robot
        local_lin_vel[:, 1] = vy_robot
        local_lin_vel[:, 2] = 0.0

        world_lin_vel = quat_apply_inverse(root_quat, local_lin_vel)

        # Set root velocity
        self._root_velocity[:, 0:3] = world_lin_vel  # Linear velocity (world frame)
        self._root_velocity[:, 3:5] = 0.0  # No roll/pitch
        self._root_velocity[:, 5] = wz  # Yaw angular velocity

        # Write velocity directly to simulation
        self._asset.write_root_velocity_to_sim(self._root_velocity)

        # Compute wheel velocities for visual animation
        # Using the same logic as convert_base_velocity_to_wheel_velocities()
        # Pass user-frame velocities (wheel animation matches user expectation)
        self._compute_wheel_velocities(vx_user, vy_user, wz)
        self._asset.set_joint_velocity_target(self._wheel_velocities, joint_ids=self._wheel_joint_ids)

    def _compute_wheel_velocities(self, vx: torch.Tensor, vy: torch.Tensor, wz: torch.Tensor):
        """Compute wheel angular velocities for visual animation.

        This replicates the logic from convert_base_velocity_to_wheel_velocities()
        in action_process.py.
        """
        r = self._wheel_radius
        R = self._base_radius

        # Scale factors (same as action_process.py)
        FORWARD_SCALE = 3.0
        STRAFE_SCALE = 1.0
        ROTATE_SCALE = 6.0

        # Forward/Backward (W/S)
        forward_left = vx / r * FORWARD_SCALE
        forward_right = -vx / r * FORWARD_SCALE
        forward_back = 0.0

        # Strafe Left/Right (A/D)
        # 60° geometry: back wheel = 1.0, front wheels = sqrt(3)/2 ≈ 0.866
        strafe_left = -vy / r * (math.sqrt(3) / 2) * STRAFE_SCALE
        strafe_right = -vy / r * (math.sqrt(3) / 2) * STRAFE_SCALE
        strafe_back = -vy / r * 1.0 * STRAFE_SCALE

        # Rotation (Z/X)
        rotate_all = wz * R / r * ROTATE_SCALE

        # Combine
        omega_left = forward_left + strafe_left - rotate_all
        omega_right = forward_right + strafe_right - rotate_all
        omega_back = forward_back + strafe_back + rotate_all

        # Back wheel axis correction
        omega_back = -omega_back

        # Output order: [back, left, right] to match action config
        self._wheel_velocities[:, 0] = omega_back
        self._wheel_velocities[:, 1] = omega_left
        self._wheel_velocities[:, 2] = omega_right


@configclass
class HolonomicBaseVelocityActionCfg(ActionTermCfg):
    """Configuration for holonomic base velocity action term."""

    class_type: type[ActionTerm] = HolonomicBaseVelocityAction

    wheel_joint_names: list[str] = MISSING
    """List of wheel joint names: [wheel_left, wheel_right, wheel_back]."""

    wheel_radius: float = 0.055
    """Radius of omni wheels in meters."""

    base_radius: float = 0.25
    """Distance from robot center to wheel axis in meters."""
