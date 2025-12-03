# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Constants for lekisaac package."""

from pathlib import Path

# Root directory of lekisaac package
LEKISAAC_ROOT = Path(__file__).parent.parent.resolve()

# URDF directory
URDF_ROOT = LEKISAAC_ROOT.parent.parent.parent / "urdf"

# LeKiwi URDF path
LEKIWI_URDF_PATH = URDF_ROOT / "lekiwi" / "lekiwi.urdf"
