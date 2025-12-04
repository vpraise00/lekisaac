# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Configuration for the LeKiwi kitchen environment."""

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

from lekisaac.assets import LEKIWI_CFG
from lekisaac.devices import init_lekiwi_action_cfg, preprocess_lekiwi_device_action

from . import mdp

# Asset paths
LEKISAAC_ASSETS_ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "assets")
KITCHEN_SCENE_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "scenes", "kiwi_kitchen.usd")
KIWI_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "objects", "Kiwi.usd")
BOWL_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "objects", "Bowl.usd")
TORUS_USD = os.path.join(LEKISAAC_ASSETS_ROOT, "objects", "Torus.usd")


@configclass
class LeKiwiKitchenSceneCfg(InteractiveSceneCfg):
    """Scene configuration for LeKiwi in kitchen environment."""

    # Invisible ground plane for robot collision
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,
                dynamic_friction=1.0,
                restitution=0.0,
            ),
            visible=False,  # Invisible - kitchen scene provides visual floor
        ),
    )

    # Kitchen scene (includes floor, walls, furniture, etc.)
    # Note: collision_enabled=False to avoid PhysX mesh cooking errors on complex geometry
    # Visual only - ground plane above provides actual collision
    # Position at z=1.38157 to align with floor level
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
                collision_enabled=False,
            ),
        ),
    )

    # Graspable objects with physics enabled
    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-1.22245, 0.14609, 0.05),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.06, 0.04, 0.04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                kinematic_enabled=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.005,
                rest_offset=0.0,
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=100.0,
                dynamic_friction=100.0,
                restitution=0.0,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        ),
    )

    bowl: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Bowl",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.04529, 0.42727, 0.03),
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=BOWL_USD,
            scale=(0.01, 0.01, 0.01),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                kinematic_enabled=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.005,
                rest_offset=0.0,
            ),
        ),
    )

    # LeKiwi robot (mobile manipulator)
    robot: ArticulationCfg = LEKIWI_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

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

    # Wrist camera
    wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/LeKiwi/Camera_Model_v3_1/wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(-0.0653, -0.8586, -0.113, 0.4957),
            convention="ros",
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

    # Base camera
    base: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/LeKiwi/Camera_Model_v3/base_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.01, 0.01, 0.0),
            rot=(0.4545, 0.5417, -0.5417, 0.4545),
            convention="ros",
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
class LeKiwiKitchenActionsCfg:
    """Action configuration for LeKiwi robot."""

    arm_action: mdp.ActionTermCfg = MISSING
    gripper_action: mdp.ActionTermCfg = MISSING
    base_velocity_action: mdp.ActionTermCfg = MISSING


@configclass
class LeKiwiKitchenEventCfg:
    """Event configuration for LeKiwi kitchen environment."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class LeKiwiKitchenObservationsCfg:
    """Observation configuration for LeKiwi kitchen environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Policy observations."""

        # Joint states
        joint_pos = ObsTerm(func=mdp.joint_pos)
        joint_vel = ObsTerm(func=mdp.joint_vel)
        actions = ObsTerm(func=mdp.last_action)

        # Camera observations
        wrist = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("wrist"), "data_type": "rgb", "normalize": False},
        )
        base = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("base"), "data_type": "rgb", "normalize": False},
        )

        # Robot pose (for mobile base)
        robot_pose = ObsTerm(func=mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_orientation = ObsTerm(func=mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_lin_vel = ObsTerm(func=mdp.root_lin_vel_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_ang_vel = ObsTerm(func=mdp.root_ang_vel_w, params={"asset_cfg": SceneEntityCfg("robot")})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class LeKiwiKitchenRewardsCfg:
    """Reward configuration (empty for teleoperation)."""

    pass


@configclass
class LeKiwiKitchenTerminationsCfg:
    """Termination configuration."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class LeKiwiKitchenEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for LeKiwi kitchen environment."""

    scene: LeKiwiKitchenSceneCfg = LeKiwiKitchenSceneCfg(env_spacing=4.0)
    observations: LeKiwiKitchenObservationsCfg = LeKiwiKitchenObservationsCfg()
    actions: LeKiwiKitchenActionsCfg = LeKiwiKitchenActionsCfg()
    events: LeKiwiKitchenEventCfg = LeKiwiKitchenEventCfg()
    rewards: LeKiwiKitchenRewardsCfg = LeKiwiKitchenRewardsCfg()
    terminations: LeKiwiKitchenTerminationsCfg = LeKiwiKitchenTerminationsCfg()
    recorders: RecordTerm = RecordTerm()

    def __post_init__(self) -> None:
        super().__post_init__()

        # Simulation settings
        self.decimation = 1
        self.episode_length_s = 60.0  # Long episode for teleoperation

        # Viewer settings (kitchen view)
        self.viewer.eye = (-2.09199, -0.61881, 0.70928)
        self.viewer.lookat = (-0.05914, 0.60306, -0.08100)

        # Physics settings
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.friction_correlation_distance = 0.00625

    def use_teleop_device(self, teleop_device: str) -> None:
        """Configure environment for specific teleoperation device.

        Args:
            teleop_device: The teleoperation device type ("lekiwi").
        """
        self.task_type = teleop_device
        self.actions = init_lekiwi_action_cfg(self.actions, device=teleop_device)

    def preprocess_device_action(self, action: dict[str, Any], teleop_device) -> torch.Tensor:
        """Preprocess device action for the environment.

        Args:
            action: Raw action from teleoperation device.
            teleop_device: The teleoperation device instance.

        Returns:
            Processed action tensor.
        """
        return preprocess_lekiwi_device_action(action, teleop_device)
