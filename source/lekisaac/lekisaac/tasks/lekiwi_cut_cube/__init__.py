# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""LeKiwi cut cube task."""

import gymnasium as gym

from .lekiwi_cut_cube_env_cfg import LeKiwiCutCubeEnvCfg

gym.register(
    id="LeKisaac-LeKiwi-CutCube-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": LeKiwiCutCubeEnvCfg,
    },
)
