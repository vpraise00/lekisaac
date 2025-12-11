# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Action processing for LeKiwi robot teleoperation."""

import math
import torch
from typing import Any

import isaaclab.envs.mdp as mdp

from lekisaac.assets import USD_JOINT_NAMES
from lekisaac.envs.actions import HolonomicBaseVelocityActionCfg


# URDF joint limits in radians (symmetric around zero)
# Same as lerobot_ros2/src/lerobot_ros2/data_collection/robot_bridge.py URDF_LIMITS
# This ensures consistent normalized → radians conversion
URDF_LIMITS = {
    "shoulder_pan": 1.91986,   # ~110°
    "shoulder_lift": 1.74533,  # ~100°
    "elbow_flex": 1.69,        # ~97°
    "wrist_flex": 1.65806,     # ~95°
    "wrist_roll": 2.74385,     # ~157°
    "gripper": 1.155855,       # ~66°
}


def init_lekiwi_action_cfg(action_cfg, device):
    """Initialize action configuration for LeKiwi device.

    Args:
        action_cfg: The action configuration object to modify.
        device: The teleoperation device type string.

    Returns:
        The modified action configuration.
    """
    if device == "lekiwi":
        # Arm actions (joint position control) - using USD joint names
        action_cfg.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                USD_JOINT_NAMES["shoulder_pan"],
                USD_JOINT_NAMES["shoulder_lift"],
                USD_JOINT_NAMES["elbow_flex"],
                USD_JOINT_NAMES["wrist_flex"],
                USD_JOINT_NAMES["wrist_roll"],
            ],
            scale=1.0,
        )
        # Gripper action (joint position control) - using USD joint name
        action_cfg.gripper_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[USD_JOINT_NAMES["gripper"]],
            scale=1.0,
        )
        # Base velocity action (holonomic) - directly sets root velocity
        # Ground friction = 0, so wheel rotation is visual only
        action_cfg.base_velocity_action = HolonomicBaseVelocityActionCfg(
            asset_name="robot",
            wheel_joint_names=[
                USD_JOINT_NAMES["wheel_left"],
                USD_JOINT_NAMES["wheel_right"],
                USD_JOINT_NAMES["wheel_back"],
            ],
            wheel_radius=0.055,
            base_radius=0.25,
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

# SO-101 Leader → SO-100 (LeKiwi) joint transformations
# These compensate for differences between SO-101 leader and SO-100 follower
# Reference: lerobot_ros2/scripts/lekiwi/lekiwi_teleop_relay.py

# Direction inversions (1.0 = same direction, -1.0 = inverted)
JOINT_INVERSIONS = {
    "shoulder_pan": 1.0,     # NOT inverted (testing - was causing vibration with -1.0)
    "shoulder_lift": -1.0,   # Inverted
    "elbow_flex": -1.0,      # Inverted
    "wrist_flex": -1.0,      # Inverted
    "wrist_roll": -1.0,      # Inverted
    "gripper": -1.0,         # Inverted
}

# Scale factors (SO-101 → LeKiwi range compensation)
# Reference: lerobot_ros2/scripts/lekiwi/lekiwi_teleop_relay.py
# Tuned empirically based on encoder range ratios and physical testing
#
# SO-100 vs SO-101 URDF Limits (from lekiwi_teleop_relay.py):
#   shoulder_lift: sim moves less than physical → scale up 1.5
#   elbow_flex: sim moves less than physical → scale up 1.5
#   wrist_flex: encoder range 2325/4095 = 0.57
#   wrist_roll: encoder range 2092/4095 = 0.51
JOINT_SCALES = {
    "shoulder_pan": 1.0,     # Direct mapping (testing for vibration issue)
    "shoulder_lift": 1.0,    # Sim moves less than physical
    "elbow_flex": 1.0,       # Increased scale for larger range
    "wrist_flex": 0.9,       # Encoder range compensation
    "wrist_roll": 0.5,       # Encoder range compensation (0.51 rounded)
    "gripper": 1.2,          # Increased for more closing range
}

# Offsets (radians) - for asymmetric calibration compensation
# Reference: lerobot_ros2/scripts/lekiwi/lekiwi_teleop_relay.py
JOINT_OFFSETS = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -1.55,   # More negative offset for earlier movement
    "elbow_flex": 1.8,       # Positive offset (reversed from previous)
    "wrist_flex": 1.3,       # Asymmetric calibration compensation
    "wrist_roll": 0.0,
    "gripper": 0.2,
}


def convert_arm_action_from_so101_leader(
    joint_state: dict[str, float],
    motor_limits: dict[str, tuple[float, float]],
    teleop_device,
) -> torch.Tensor:
    """Convert SO101Leader joint state to arm action tensor.

    Uses the same conversion approach as lerobot_ros2's robot_bridge.py:
    1. Normalized position (-100 to 100) → radians using URDF_LIMITS (symmetric)
    2. Apply SO-101 → SO-100 (LeKiwi) transformations (inversions, scales, offsets)

    Args:
        joint_state: Dictionary of normalized joint positions from SO101Leader.
                     Arm joints: -100 to 100, Gripper: 0 to 100.
        motor_limits: Dictionary of motor position limits (unused, kept for API compatibility).
        teleop_device: The teleoperation device instance.

    Returns:
        Tensor of shape (num_envs, 6) containing arm joint positions in radians.
    """
    processed_action = torch.zeros(teleop_device.env.num_envs, 6, device=teleop_device.env.device)

    # Debug: track shoulder_pan values (set to True to enable)
    debug_shoulder_pan = False

    for joint_name, motor_id in joint_names_to_motor_ids.items():
        # Get normalized position from leader
        normalized_pos = joint_state[joint_name]

        # Convert normalized → radians using symmetric URDF limits
        # Same as lerobot_ros2/robot_bridge.py: radians = normalized / 100.0 * urdf_limit
        urdf_limit = URDF_LIMITS.get(joint_name, 1.75)

        if joint_name == "gripper":
            # Gripper uses 0-100 range, map to 0 to urdf_limit
            processed_radius = normalized_pos / 100.0 * urdf_limit
        else:
            # Arm joints use -100 to 100 range, map to -urdf_limit to +urdf_limit
            processed_radius = normalized_pos / 100.0 * urdf_limit

        # Apply SO-101 → SO-100 (LeKiwi) transformations
        # Formula: cmd = pos * inversion * scale + offset
        # Reference: lerobot_ros2/scripts/lekiwi/lekiwi_teleop_relay.py
        inversion = JOINT_INVERSIONS.get(joint_name, 1.0)
        scale = JOINT_SCALES.get(joint_name, 1.0)
        offset = JOINT_OFFSETS.get(joint_name, 0.0)

        transformed_radius = processed_radius * inversion * scale + offset
        processed_action[:, motor_id] = transformed_radius

        # Debug output for shoulder_pan
        if joint_name == "shoulder_pan" and debug_shoulder_pan:
            print(f"[DEBUG] shoulder_pan: norm={normalized_pos:.2f}, rad={processed_radius:.3f}, "
                  f"inv={inversion}, scale={scale}, offset={offset}, final={transformed_radius:.3f}")

    return processed_action


def convert_base_velocity_to_wheel_velocities(
    base_velocity: torch.Tensor,
    wheel_radius: float = 0.055,  # LeKiwi omni wheel radius in meters
    base_radius: float = 0.25,    # Distance from base center to wheel axis
) -> torch.Tensor:
    """Convert base velocity command to individual wheel velocities.

    LeKiwi Y-configuration kiwi drive (looking from above, arm pointing +X):
    - ST3215_Servo_Motor_v1_2 (Revolute_60) = Back wheel (axis: 0, 0, -1)
    - ST3215_Servo_Motor_v1_1 (Revolute_62) = Right front wheel (axis: 0.866, 0, 0.5)
    - ST3215_Servo_Motor_v1 (Revolute_64) = Left front wheel (axis: -0.866, 0, 0.5)

    Motion control:
    - W/S (vx): Forward/backward - front wheels rotate opposite, back minimal
    - A/D (vy): Strafe left/right - all wheels same direction, ratio back:front = 1 : 2/√3
    - Z/X (wz): Rotation - all wheels same direction uniformly (Z=CW, X=CCW)

    Args:
        base_velocity: Tensor of shape (num_envs, 3) with [linear_x, linear_y, angular_z].
        wheel_radius: Radius of each wheel in meters.
        base_radius: Distance from base center to wheel axis.

    Returns:
        Tensor of shape (num_envs, 3) with wheel angular velocities [left, right, back].
    """
    # Extract velocity components
    vx = base_velocity[:, 0]  # Forward velocity (+X)
    vy = base_velocity[:, 1]  # Left strafe velocity (+Y)
    wz = base_velocity[:, 2]  # CCW rotation (positive = CCW robot rotation)

    r = wheel_radius
    R = base_radius

    # Motion-specific scale factors
    FORWARD_SCALE = 3.0   # W/S: 1.5x
    STRAFE_SCALE = 1.0    # A/D: reduced to prevent base lift
    ROTATE_SCALE = 6.0    # Z/X: 6x

    # === Forward/Backward (W/S) ===
    # Direct velocity without kinematic factor (user request: no 2/sqrt3)
    forward_left = vx / r * FORWARD_SCALE
    forward_right = -vx / r * FORWARD_SCALE
    forward_back = 0.0

    # === Strafe Left/Right (A/D) ===
    # 60° angle geometry: front wheels at 60° from Y-axis, back wheel aligned with Y
    # Front wheel compensation: 1/cos(60°) = 2.0, Back wheel: 1.0
    strafe_left = -vy / r * 2.0 * STRAFE_SCALE
    strafe_right = -vy / r * 2.0 * STRAFE_SCALE
    strafe_back = -vy / r * 1.0 * STRAFE_SCALE

    # === Rotation (Z/X) ===
    # 6x speed (3배 증가)
    rotate_all = wz * R / r * ROTATE_SCALE

    # Combine all components
    # Front wheels need inverted rotation direction
    omega_left = forward_left + strafe_left - rotate_all
    omega_right = forward_right + strafe_right - rotate_all
    omega_back = forward_back + strafe_back + rotate_all

    # === Physical Wheel Mapping (determined by forward motion observation) ===
    # When W pressed: back & left_front rotating, right_front NOT rotating
    # My output [pos, pos, 0] means:
    #   - Position 0 (Revolute_64) = back wheel (getting omega that should be 0)
    #   - Position 1 (Revolute_62) = left front wheel (rotating correctly)
    #   - Position 2 (Revolute_60) = right front wheel (getting 0, should be rotating)
    #
    # URDF Axes:
    #   - Revolute_64 (back): axis (-0.866, 0, 0.5)
    #   - Revolute_62 (left front): axis (0.866, 0, 0.5)
    #   - Revolute_60 (right front): axis (0, 0, -1)
    #
    # Axis corrections:
    #   - Back (Revolute_64, axis -0.866,0,0.5): needs inversion
    #   - Left front (Revolute_62, axis 0.866,0,0.5): no inversion
    #   - Right front (Revolute_60, axis 0,0,-1): no inversion
    omega_back = -omega_back  # Back wheel axis has negative X component

    # Note: scales are applied per-motion above (FORWARD_SCALE, STRAFE_SCALE, ROTATE_SCALE)

    # Stack into output tensor to match action config order
    # Action config: [Revolute_64, Revolute_62, Revolute_60]
    # Physical:      [back,        left_front,  right_front]
    # Output:        [omega_back,  omega_left,  omega_right]
    wheel_velocities = torch.stack([omega_back, omega_left, omega_right], dim=-1)

    return wheel_velocities


def preprocess_lekiwi_device_action(action: dict[str, Any], teleop_device) -> torch.Tensor:
    """Process LeKiwi device action into environment action tensor.

    Args:
        action: Dictionary containing device action data.
        teleop_device: The teleoperation device instance.

    Returns:
        Tensor containing the processed action for the environment.
        Shape: (num_envs, 9) = [arm(5) + gripper(1) + base_velocity(3)]
    """
    if action.get("lekiwi_device") is not None:
        # Process arm action from SO101Leader
        arm_action = convert_arm_action_from_so101_leader(
            action["joint_state"],
            action["motor_limits"],
            teleop_device,
        )

        # Process base velocity from keyboard [vx, vy, wz]
        # Pass directly to HolonomicBaseVelocityAction (no wheel conversion here)
        base_velocity = torch.as_tensor(
            action["base_velocity"],
            device=teleop_device.env.device,
            dtype=torch.float32,
        ).clone()
        base_velocity = base_velocity.unsqueeze(0).expand(teleop_device.env.num_envs, -1)

        # Combine arm (6 DOF) + base velocity (3 DOF) actions
        # arm_action[:, :5] = arm joints, arm_action[:, 5] = gripper
        # base_velocity[:, :3] = [vx, vy, wz] for HolonomicBaseVelocityAction
        processed_action = torch.cat([arm_action, base_velocity], dim=-1)

        return processed_action
    else:
        raise NotImplementedError("Only lekiwi_device teleoperation is supported.")
