# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Configuration for the LeKiwi Robot (mobile manipulator with SO-101 arm)."""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from lekisaac.utils.constant import URDF_ROOT


# LeKiwi USD asset paths (smooth = cylinder wheel collision for stable movement)
LEKIWI_ASSET_PATH = Path(URDF_ROOT) / "lekiwi" / "lekiwi_smooth.usd"
LEKIWI_AUGMENTED_ASSET_PATH = Path(URDF_ROOT) / "lekiwi" / "lekiwi_elevated_smooth.usd"  # 0.8m taller

# Joint name mapping from USD (actual names from URDF conversion)
# Arm joints:
#   STS3215_03a_v1_Revolute_45 -> shoulder_pan
#   STS3215_03a_v1_1_Revolute_49 -> shoulder_lift
#   STS3215_03a_v1_2_Revolute_51 -> elbow_flex
#   STS3215_03a_v1_3_Revolute_53 -> wrist_flex
#   STS3215_03a_Wrist_Roll_v1_Revolute_55 -> wrist_roll
#   STS3215_03a_v1_4_Revolute_57 -> gripper
# Wheel joints:
#   ST3215_Servo_Motor_v1_Revolute_64 -> wheel_left
#   ST3215_Servo_Motor_v1_1_Revolute_62 -> wheel_right
#   ST3215_Servo_Motor_v1_2_Revolute_60 -> wheel_back

# USD joint names (as exported from URDF)
USD_JOINT_NAMES = {
    "shoulder_pan": "STS3215_03a_v1_Revolute_45",
    "shoulder_lift": "STS3215_03a_v1_1_Revolute_49",
    "elbow_flex": "STS3215_03a_v1_2_Revolute_51",
    "wrist_flex": "STS3215_03a_v1_3_Revolute_53",
    "wrist_roll": "STS3215_03a_Wrist_Roll_v1_Revolute_55",
    "gripper": "STS3215_03a_v1_4_Revolute_57",
    "wheel_left": "ST3215_Servo_Motor_v1_Revolute_64",
    "wheel_right": "ST3215_Servo_Motor_v1_1_Revolute_62",
    "wheel_back": "ST3215_Servo_Motor_v1_2_Revolute_60",
}

# LeKiwi robot configuration
# Note: fix_root_link=False to allow mobile base movement
LEKIWI_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(LEKIWI_ASSET_PATH),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True,
            contact_offset=0.005,
            rest_offset=0.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=8,
            fix_root_link=False,  # Mobile base - unfixed root
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.05),  # Start slightly above ground for wheels
        rot=(1.0, 0.0, 0.0, 0.0),  # wxyz quaternion
        joint_pos={
            # Arm joints (using actual USD joint names)
            USD_JOINT_NAMES["shoulder_pan"]: 0.0,
            USD_JOINT_NAMES["shoulder_lift"]: 0.0,
            USD_JOINT_NAMES["elbow_flex"]: 0.0,
            USD_JOINT_NAMES["wrist_flex"]: 0.0,
            USD_JOINT_NAMES["wrist_roll"]: 0.0,
            USD_JOINT_NAMES["gripper"]: 0.0,
            # Wheel joints (continuous)
            USD_JOINT_NAMES["wheel_left"]: 0.0,
            USD_JOINT_NAMES["wheel_right"]: 0.0,
            USD_JOINT_NAMES["wheel_back"]: 0.0,
        },
    ),
    actuators={
        # Gripper actuator - matching leisaac SO101 settings
        "sts3215-gripper": ImplicitActuatorCfg(
            joint_names_expr=[USD_JOINT_NAMES["gripper"]],
            effort_limit_sim=10,
            velocity_limit_sim=10,
            stiffness=17.8,
            damping=0.60,
        ),
        # Arm actuators (5 DOF)
        # Higher stiffness/damping to resist inertial forces during base movement
        "sts3215-arm": ImplicitActuatorCfg(
            joint_names_expr=[
                USD_JOINT_NAMES["shoulder_pan"],
                USD_JOINT_NAMES["shoulder_lift"],
                USD_JOINT_NAMES["elbow_flex"],
                USD_JOINT_NAMES["wrist_flex"],
                USD_JOINT_NAMES["wrist_roll"],
            ],
            effort_limit_sim=10,
            velocity_limit_sim=10,
            stiffness=40.0,
            damping=8.0,
        ),
        # Wheel actuators for omni-directional movement
        # For velocity control: stiffness=0, damping must be VERY HIGH
        # Reference: https://docs.isaacsim.omniverse.nvidia.com/latest/robot_simulation/mobile_robot_controllers.html
        "wheel-motors": ImplicitActuatorCfg(
            joint_names_expr=[
                USD_JOINT_NAMES["wheel_left"],
                USD_JOINT_NAMES["wheel_right"],
                USD_JOINT_NAMES["wheel_back"],
            ],
            effort_limit_sim=100000,  # Increased 500x for heavy base (80kg)
            velocity_limit_sim=50,
            stiffness=0.0,  # Velocity control mode - MUST be 0
            damping=5e5,    # Increased 5x for stronger velocity response
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


# LeKiwi Augmented configuration (0.8m taller arm for elevated tasks)
# Uses lekiwi_augmented.usd with modified Rigid_21 joint (z: -0.05 -> -0.85)
LEKIWI_AUGMENTED_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(LEKIWI_AUGMENTED_ASSET_PATH),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True,
            contact_offset=0.005,
            rest_offset=0.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=8,
            fix_root_link=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.05),  # Same base height, arm is 0.8m taller internally
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            USD_JOINT_NAMES["shoulder_pan"]: 0.0,
            USD_JOINT_NAMES["shoulder_lift"]: 0.0,
            USD_JOINT_NAMES["elbow_flex"]: 0.0,
            USD_JOINT_NAMES["wrist_flex"]: 0.0,
            USD_JOINT_NAMES["wrist_roll"]: 0.0,
            USD_JOINT_NAMES["gripper"]: 0.0,
            USD_JOINT_NAMES["wheel_left"]: 0.0,
            USD_JOINT_NAMES["wheel_right"]: 0.0,
            USD_JOINT_NAMES["wheel_back"]: 0.0,
        },
    ),
    actuators={
        # Gripper actuator - matching leisaac SO101 settings
        "sts3215-gripper": ImplicitActuatorCfg(
            joint_names_expr=[USD_JOINT_NAMES["gripper"]],
            effort_limit_sim=10,
            velocity_limit_sim=10,
            stiffness=17.8,
            damping=0.60,
        ),
        # Arm actuators (5 DOF)
        # Higher stiffness/damping to resist inertial forces during base movement
        "sts3215-arm": ImplicitActuatorCfg(
            joint_names_expr=[
                USD_JOINT_NAMES["shoulder_pan"],
                USD_JOINT_NAMES["shoulder_lift"],
                USD_JOINT_NAMES["elbow_flex"],
                USD_JOINT_NAMES["wrist_flex"],
                USD_JOINT_NAMES["wrist_roll"],
            ],
            effort_limit_sim=10,
            velocity_limit_sim=10,
            stiffness=40.0,
            damping=8.0,
        ),
        "wheel-motors": ImplicitActuatorCfg(
            joint_names_expr=[
                USD_JOINT_NAMES["wheel_left"],
                USD_JOINT_NAMES["wheel_right"],
                USD_JOINT_NAMES["wheel_back"],
            ],
            effort_limit_sim=100000,
            velocity_limit_sim=50,
            stiffness=0.0,
            damping=5e5,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
