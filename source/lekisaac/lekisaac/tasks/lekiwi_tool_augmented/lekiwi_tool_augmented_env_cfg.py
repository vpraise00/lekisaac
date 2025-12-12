# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Configuration for the LeKiwi tool augmented environment.

This environment uses a LeKiwi robot with an elevated arm (0.8m taller)
for tasks requiring manipulation at higher surfaces (e.g., tables, counters).
"""

import os
import torch
from dataclasses import MISSING
from typing import Any

import isaaclab.sim as sim_utils
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg as RecordTerm
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
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


@configclass
class LeKiwiToolAugmentedSceneCfg(InteractiveSceneCfg):
    """Scene configuration for LeKiwi tool augmented environment."""

    # Invisible ground plane for robot collision
    # Friction = 0 for velocity-based holonomic control (wheel rotation is visual only)
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

    # Spatula on kitchen counter surface
    spatula: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Spatula",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.97, 0.91, 1.0),  # On kitchen counter (raised to avoid collision)
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=SPATULA_USD,
            scale=(0.01, 0.01, 0.02),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                kinematic_enabled=False,
                max_depenetration_velocity=0.5,  # Reduced for stable grasping
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),  # Lighter for easier grasping
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,  # Increased for better contact detection
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
class LeKiwiToolAugmentedActionsCfg:
    """Action configuration for LeKiwi robot."""

    arm_action: mdp.ActionTermCfg = MISSING
    gripper_action: mdp.ActionTermCfg = MISSING
    base_velocity_action: mdp.ActionTermCfg = MISSING


@configclass
class LeKiwiToolAugmentedEventCfg:
    """Event configuration."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class LeKiwiToolAugmentedObservationsCfg:
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
class LeKiwiToolAugmentedRewardsCfg:
    """Reward configuration (empty for teleoperation)."""

    pass


@configclass
class LeKiwiToolAugmentedTerminationsCfg:
    """Termination configuration."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class LeKiwiToolAugmentedEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for LeKiwi tool augmented environment."""

    scene: LeKiwiToolAugmentedSceneCfg = LeKiwiToolAugmentedSceneCfg(env_spacing=4.0)
    observations: LeKiwiToolAugmentedObservationsCfg = LeKiwiToolAugmentedObservationsCfg()
    actions: LeKiwiToolAugmentedActionsCfg = LeKiwiToolAugmentedActionsCfg()
    events: LeKiwiToolAugmentedEventCfg = LeKiwiToolAugmentedEventCfg()
    rewards: LeKiwiToolAugmentedRewardsCfg = LeKiwiToolAugmentedRewardsCfg()
    terminations: LeKiwiToolAugmentedTerminationsCfg = LeKiwiToolAugmentedTerminationsCfg()
    recorders: RecordTerm = RecordTerm()

    def __post_init__(self) -> None:
        super().__post_init__()

        self.decimation = 1
        self.episode_length_s = 60.0

        # Viewer settings (elevated view for tool augmented)
        self.viewer.eye = (-2.0, -1.0, 2.0)
        self.viewer.lookat = (0.0, 0.0, 1.0)

        # Physics settings for stable grasping
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.friction_correlation_distance = 0.00625
        self.sim.physx.gpu_max_rigid_patch_count = 2**20
        # Increase solver iterations for stable contact during grasping
        self.sim.physx.solver_position_iteration_count = 32
        self.sim.physx.solver_velocity_iteration_count = 16

        # Note: Do NOT set sim.physics_material here as it affects robot wheels too
        # Object friction is set per-object in scene config instead

    def use_teleop_device(self, teleop_device: str) -> None:
        """Configure environment for specific teleoperation device."""
        self.task_type = teleop_device
        self.actions = init_lekiwi_action_cfg(self.actions, device=teleop_device)

    def preprocess_device_action(self, action: dict[str, Any], teleop_device) -> torch.Tensor:
        """Preprocess device action for the environment."""
        return preprocess_lekiwi_device_action(action, teleop_device)
