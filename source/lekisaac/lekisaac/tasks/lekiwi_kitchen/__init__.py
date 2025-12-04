# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""LeKiwi kitchen task registration."""

import gymnasium as gym

gym.register(
    id="LeKisaac-LeKiwi-Kitchen-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.lekiwi_kitchen_env_cfg:LeKiwiKitchenEnvCfg",
    },
)
