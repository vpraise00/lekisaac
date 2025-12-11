# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Configuration for the LeKiwi teleoperation environment."""

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
from isaaclab.sensors import TiledCameraCfg, FrameTransformerCfg, OffsetCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from lekisaac.assets import LEKIWI_CFG
from lekisaac.devices import init_lekiwi_action_cfg, preprocess_lekiwi_device_action

from . import mdp


@configclass
class LeKiwiTeleopSceneCfg(InteractiveSceneCfg):
    """Scene configuration for LeKiwi teleoperation."""

    # Ground plane with zero friction for velocity-based holonomic control
    # Wheel rotation is for visual animation only; base movement is controlled directly
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.0,
                dynamic_friction=0.0,
                restitution=0.0,
            ),
        ),
    )

    # LeKiwi robot (mobile manipulator)
    # Note: USD structure is /Root/LeKiwi/... so robot prims are at {ENV_REGEX_NS}/Robot/LeKiwi/...
    robot: ArticulationCfg = LEKIWI_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # End-effector frame transformer
    # base_plate_layer1_v5 is the articulation root, Moving_Jaw_08d_v1 is the gripper
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
    # Transform from set_lekiwi_cameras.py: translate (0, 0, 0), rotateXYZ (-15, -120, 0)
    wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/LeKiwi/Camera_Model_v3_1/wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(-0.0653, -0.8586, -0.113, 0.4957),  # XYZ Euler (-15, -120, 0) -> quaternion (xyzw)
            convention="ros",
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=18.0,  # From set_lekiwi_cameras.py
            focus_distance=1.0,
            horizontal_aperture=20.955,  # From set_lekiwi_cameras.py
            clipping_range=(0.01, 200.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,
    )

    # Base camera (attached to Camera_Model_v3 - base camera model)
    # Transform from set_lekiwi_cameras.py: translate (-0.01, 0.01, 0), rotateXYZ (90, 100, 0)
    base: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/LeKiwi/Camera_Model_v3/base_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.01, 0.01, 0.0),
            rot=(0.4545, 0.5417, -0.5417, 0.4545),  # XYZ Euler (90, 100, 0) -> quaternion (xyzw)
            convention="ros",
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=18.0,  # From set_lekiwi_cameras.py
            focus_distance=1.0,
            horizontal_aperture=20.955,  # From set_lekiwi_cameras.py
            clipping_range=(0.01, 200.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,
    )

    # Dome light
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # Small cube for grasping test
    # Placed in front of the robot gripper (robot starts at origin, arm extends forward)
    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.02, 0.02, 0.02),  # 2cm cube
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=1.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),  # 20g (slightly heavier for stability)
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.005,  # 5mm - detect contacts earlier for grasping
                rest_offset=0.0,       # No penetration at rest
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,   # High friction for better grip
                dynamic_friction=1.0,
                restitution=0.0,       # No bounce
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.2, 0.2),  # Red color for visibility
                metallic=0.2,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0247, 0.29177, 0.12),  # Near gripper position
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )


@configclass
class LeKiwiActionsCfg:
    """Action configuration for LeKiwi robot."""

    arm_action: mdp.ActionTermCfg = MISSING
    gripper_action: mdp.ActionTermCfg = MISSING
    base_velocity_action: mdp.ActionTermCfg = MISSING


@configclass
class LeKiwiEventCfg:
    """Event configuration for LeKiwi teleoperation."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class LeKiwiObservationsCfg:
    """Observation configuration for LeKiwi teleoperation."""

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
class LeKiwiRewardsCfg:
    """Reward configuration (empty for teleoperation)."""

    pass


@configclass
class LeKiwiTerminationsCfg:
    """Termination configuration."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class LeKiwiTeleopEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for LeKiwi teleoperation environment."""

    scene: LeKiwiTeleopSceneCfg = LeKiwiTeleopSceneCfg(env_spacing=4.0)
    observations: LeKiwiObservationsCfg = LeKiwiObservationsCfg()
    actions: LeKiwiActionsCfg = LeKiwiActionsCfg()
    events: LeKiwiEventCfg = LeKiwiEventCfg()
    rewards: LeKiwiRewardsCfg = LeKiwiRewardsCfg()
    terminations: LeKiwiTerminationsCfg = LeKiwiTerminationsCfg()
    recorders: RecordTerm = RecordTerm()

    def __post_init__(self) -> None:
        super().__post_init__()

        # Simulation settings
        self.decimation = 1
        self.episode_length_s = 60.0  # Long episode for teleoperation

        # Viewer settings
        self.viewer.eye = (2.0, 2.0, 2.0)
        self.viewer.lookat = (0.0, 0.0, 0.5)

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
