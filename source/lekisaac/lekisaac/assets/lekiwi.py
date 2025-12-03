# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Configuration for the LeKiwi Robot (mobile manipulator with SO-101 arm)."""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from lekisaac.utils.constant import URDF_ROOT


# LeKiwi USD asset path (to be created from URDF conversion)
# For now, we use a placeholder path - users need to convert URDF to USD
LEKIWI_ASSET_PATH = Path(URDF_ROOT) / "lekiwi" / "lekiwi.usd"

# LeKiwi robot configuration
# Note: fix_root_link=False to allow mobile base movement
LEKIWI_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(LEKIWI_ASSET_PATH),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
            fix_root_link=False,  # Mobile base - unfixed root
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.05),  # Start slightly above ground for wheels
        rot=(1.0, 0.0, 0.0, 0.0),  # wxyz quaternion
        joint_pos={
            # Arm joints (same as SO101)
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
            # Wheel joints (continuous)
            "wheel_left": 0.0,
            "wheel_right": 0.0,
            "wheel_back": 0.0,
        },
    ),
    actuators={
        # Gripper actuator
        "sts3215-gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper"],
            effort_limit_sim=10,
            velocity_limit_sim=10,
            stiffness=17.8,
            damping=0.60,
        ),
        # Arm actuators (5 DOF)
        "sts3215-arm": ImplicitActuatorCfg(
            joint_names_expr=[
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ],
            effort_limit_sim=10,
            velocity_limit_sim=10,
            stiffness=17.8,
            damping=0.60,
        ),
        # Wheel actuators for omni-directional movement
        "wheel-motors": ImplicitActuatorCfg(
            joint_names_expr=["wheel_.*"],
            effort_limit_sim=20,
            velocity_limit_sim=20,
            stiffness=0.0,  # Velocity control mode
            damping=10.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)


# Joint limits written in USD (degrees) - same as SO101 for arm joints
LEKIWI_USD_JOINT_LIMITS = {
    # Arm joints
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 90.0),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-160.0, 160.0),
    "gripper": (-10.0, 100.0),
    # Wheel joints (continuous - no limits)
    "wheel_left": (-float("inf"), float("inf")),
    "wheel_right": (-float("inf"), float("inf")),
    "wheel_back": (-float("inf"), float("inf")),
}


# Motor limits written in real device (normalized to related range)
LEKIWI_MOTOR_LIMITS = {
    # Arm motor limits (same as SO101)
    "shoulder_pan": (-100.0, 100.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 100.0),
    "wrist_flex": (-100.0, 100.0),
    "wrist_roll": (-100.0, 100.0),
    "gripper": (0.0, 100.0),
}


# Base velocity limits (m/s for linear, rad/s for angular)
LEKIWI_BASE_VELOCITY_LIMITS = {
    "linear_x": (-0.5, 0.5),   # Forward/backward
    "linear_y": (-0.5, 0.5),   # Left/right strafe (omni-directional)
    "angular_z": (-1.0, 1.0),  # Rotation
}
