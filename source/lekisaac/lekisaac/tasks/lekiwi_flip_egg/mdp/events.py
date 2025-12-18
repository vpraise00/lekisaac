# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Custom event functions for LeKiwi flip egg task.

Note: Per-body friction is now handled by using ArticulationCfg for the spatula
with separate Handle and Blade bodies, each targeted by randomize_rigid_body_material
with body_names parameter.
"""

# No custom events needed - using IsaacLab's randomize_rigid_body_material
# with body_names to target specific bodies of the spatula articulation.
