# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Configuration for the LeKiwi flip egg environment.

This environment uses a LeKiwi robot with an elevated arm (0.8m taller)
for flipping an egg on a frying pan using a spatula.

Scene setup:
- Frying pan (pan_B) on gas range (fixed, kinematic - no gravity/physics)
- Egg on the frying pan (dynamic, needs to be flipped)
- Spatula for the robot to grasp and use
"""

import os
import torch
from dataclasses import MISSING
from typing import Any

import isaaclab.sim as sim_utils
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg as RecordTerm
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp import randomize_rigid_body_material
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import TiledCameraCfg, FrameTransformerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from lekisaac.assets import LEKIWI_AUGMENTED_CFG
from lekisaac.devices import init_lekiwi_action_cfg, preprocess_lekiwi_device_action

from . import mdp

# Asset paths
LEKISAAC_ASSETS_ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "assets")
KITCHEN_SCENE_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "scenes", "kiwi_kitchen.usd")
SPATULA_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "objects", "Spatula.usd")
SPATULA_ARTICULATION_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "objects", "Spatula_articulation.usd")
PAN_B_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "objects", "pan_B.usd")
EGG_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "objects", "Egg.usd")

# Stove/burner position in kitchen (calculated from USD)
# Stove world pos in IsaacLab: (0.400, 1.045, 0.509) with kitchen at z=1.38157
STOVE_POS = (0.194, 0.95, 0.954)  # Front-left burner position


@configclass
class LeKiwiFlipEggSceneCfg(InteractiveSceneCfg):
    """Scene configuration for LeKiwi flip egg environment."""

    # Invisible ground plane for robot collision (zero friction for wheels)
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.0,
                dynamic_friction=0.0,
                restitution=0.0,
            ),
            visible=False,
        ),
    )

    # Kitchen scene with collision enabled for counter/table surfaces
    kitchen = AssetBaseCfg(
        prim_path="/World/Kitchen",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.38157),
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=KITCHEN_SCENE_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                kinematic_enabled=True,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
        ),
    )

    # Spatula (tool) - RigidObject
    spatula: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Spatula",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.97, 0.91, 1.0),  # On kitchen counter
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=SPATULA_USD,
            scale=(0.008, 0.008, 0.016),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                kinematic_enabled=False,
                max_depenetration_velocity=0.5,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.01),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.005,
                rest_offset=0.0,
            ),
        ),
    )

    # Frying pan on gas range (fixed position, kinematic)
    # - Kinematic: True (no gravity, no physics response - stays fixed)
    # - Collision: enabled (so egg and spatula can interact with it)
    frying_pan: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/FryingPan",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=STOVE_POS,  # On gas range front-left burner
            rot=(0.0, 0.0, 0.0, 1.0),  # 180 degrees around Z axis (quaternion: w, x, y, z)
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=PAN_B_USD,
            scale=(0.01, 0.01, 0.01),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                kinematic_enabled=True,  # Fixed in place - won't move or fall
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
        ),
    )

    # Egg on the frying pan (dynamic, needs to be flipped)
    egg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Egg",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(STOVE_POS[0], STOVE_POS[1], STOVE_POS[2] + 0.05),  # Slightly above frying pan
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=EGG_USD,
            scale=(0.1, 0.1, 0.01),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                kinematic_enabled=False,
                max_depenetration_velocity=0.5,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),  # 50g egg
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.005,
                rest_offset=0.0,
            ),
        ),
    )

    # LeKiwi robot with augmented height (0.8m taller arm)
    robot: ArticulationCfg = LEKIWI_AUGMENTED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # End-effector frame transformer
    ee_frame: FrameTransformerCfg = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/LeKiwi/base_plate_layer1_v5",
        debug_vis=False,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/LeKiwi/Moving_Jaw_08d_v1",
                name="gripper",
            ),
        ],
    )

    # Wrist camera (attached to Camera_Model_v3_1 - wrist camera model)
    wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/LeKiwi/Camera_Model_v3_1/wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=18.0,
            focus_distance=1.0,
            horizontal_aperture=20.955,
            clipping_range=(0.01, 200.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,
    )

    # Base camera (attached to Camera_Model_v3 - base camera model)
    base: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/LeKiwi/Camera_Model_v3/base_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=18.0,
            focus_distance=1.0,
            horizontal_aperture=20.955,
            clipping_range=(0.01, 200.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,
    )

    # Lighting
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


@configclass
class LeKiwiFlipEggActionsCfg:
    """Action configuration for LeKiwi robot."""

    arm_action: mdp.ActionTermCfg = MISSING
    gripper_action: mdp.ActionTermCfg = MISSING
    base_velocity_action: mdp.ActionTermCfg = MISSING


@configclass
class LeKiwiFlipEggEventCfg:
    """Event configuration."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    # Set friction on spatula
    set_spatula_friction = EventTerm(
        func=randomize_rigid_body_material,
        mode="startup",
        params={
            "static_friction_range": (3.0, 3.0),
            "dynamic_friction_range": (3.0, 3.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 1,
            "asset_cfg": SceneEntityCfg("spatula"),
        },
    )

    # Set high friction on robot gripper to hold spatula
    set_gripper_friction = EventTerm(
        func=randomize_rigid_body_material,
        mode="startup",
        params={
            "static_friction_range": (15.0, 15.0),
            "dynamic_friction_range": (15.0, 15.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 1,
            "asset_cfg": SceneEntityCfg("robot", body_names=["Moving_Jaw_08d_v1"]),
        },
    )


@configclass
class LeKiwiFlipEggObservationsCfg:
    """Observation configuration."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Policy observations."""

        joint_pos = ObsTerm(func=mdp.joint_pos)
        joint_vel = ObsTerm(func=mdp.joint_vel)
        actions = ObsTerm(func=mdp.last_action)

        wrist = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("wrist"), "data_type": "rgb", "normalize": False},
        )
        base = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("base"), "data_type": "rgb", "normalize": False},
        )

        robot_pose = ObsTerm(func=mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_orientation = ObsTerm(func=mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_lin_vel = ObsTerm(func=mdp.root_lin_vel_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_ang_vel = ObsTerm(func=mdp.root_ang_vel_w, params={"asset_cfg": SceneEntityCfg("robot")})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class LeKiwiFlipEggRewardsCfg:
    """Reward configuration (empty for teleoperation)."""

    pass


@configclass
class LeKiwiFlipEggTerminationsCfg:
    """Termination configuration."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class LeKiwiFlipEggEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for LeKiwi flip egg environment."""

    scene: LeKiwiFlipEggSceneCfg = LeKiwiFlipEggSceneCfg(env_spacing=4.0)
    observations: LeKiwiFlipEggObservationsCfg = LeKiwiFlipEggObservationsCfg()
    actions: LeKiwiFlipEggActionsCfg = LeKiwiFlipEggActionsCfg()
    events: LeKiwiFlipEggEventCfg = LeKiwiFlipEggEventCfg()
    rewards: LeKiwiFlipEggRewardsCfg = LeKiwiFlipEggRewardsCfg()
    terminations: LeKiwiFlipEggTerminationsCfg = LeKiwiFlipEggTerminationsCfg()
    recorders: RecordTerm = RecordTerm()

    def __post_init__(self) -> None:
        super().__post_init__()

        self.decimation = 1
        self.episode_length_s = 60.0

        # Viewer settings (elevated view for flip egg task)
        self.viewer.eye = (-2.0, -1.0, 2.0)
        self.viewer.lookat = (0.0, 0.0, 1.0)

        # Physics settings for stable grasping and tool manipulation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.friction_correlation_distance = 0.00625
        self.sim.physx.gpu_max_rigid_patch_count = 2**20
        # Solver iterations (lower = better performance, higher = more stable grasping)
        self.sim.physx.solver_position_iteration_count = 8
        self.sim.physx.solver_velocity_iteration_count = 4

        # Note: No global physics_material - friction is set per-object via randomize_rigid_body_material event

    def use_teleop_device(self, teleop_device: str) -> None:
        """Configure environment for specific teleoperation device."""
        self.task_type = teleop_device
        self.actions = init_lekiwi_action_cfg(self.actions, device=teleop_device)

    def preprocess_device_action(self, action: dict[str, Any], teleop_device) -> torch.Tensor:
        """Preprocess device action for the environment."""
        return preprocess_lekiwi_device_action(action, teleop_device)
