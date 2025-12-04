# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Script to convert LeKiwi URDF to USD format for IsaacLab."""

from isaacsim import SimulationApp

# Must create SimulationApp first
simulation_app = SimulationApp({"headless": True})

import argparse
from pathlib import Path


def convert_urdf_to_usd(urdf_path: str, usd_path: str, fix_base: bool = False):
    """Convert URDF file to USD format.

    Args:
        urdf_path: Path to input URDF file.
        usd_path: Path to output USD file.
        fix_base: Whether to fix the robot base (False for mobile robots).
    """
    from omni.isaac.urdf import _urdf

    urdf_interface = _urdf.acquire_urdf_interface()

    # Configure import settings
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False  # Keep fixed joints for structure
    import_config.fix_base = fix_base  # False for mobile base
    import_config.import_inertia_tensor = True
    import_config.self_collision = True
    import_config.default_drive_type = 1  # Position drive
    import_config.default_drive_strength = 1e4
    import_config.default_position_drive_damping = 1e3

    print(f"Parsing URDF: {urdf_path}")
    result = urdf_interface.parse_urdf(urdf_path, import_config)

    if result:
        print(f"Importing robot to USD: {usd_path}")
        urdf_interface.import_robot(usd_path, result, import_config, "lekiwi")
        print(f"Successfully created: {usd_path}")
    else:
        print("Failed to parse URDF!")


def main():
    parser = argparse.ArgumentParser(description="Convert LeKiwi URDF to USD")
    parser.add_argument(
        "--urdf",
        type=str,
        default=None,
        help="Path to URDF file (default: lekisaac/urdf/lekiwi/lekiwi.urdf)",
    )
    parser.add_argument(
        "--usd",
        type=str,
        default=None,
        help="Path to output USD file (default: lekisaac/urdf/lekiwi/lekiwi.usd)",
    )
    parser.add_argument(
        "--fix-base",
        action="store_true",
        help="Fix the robot base (default: False for mobile robots)",
    )
    args = parser.parse_args()

    # Default paths
    script_dir = Path(__file__).parent.parent
    default_urdf = script_dir / "urdf" / "lekiwi" / "lekiwi.urdf"
    default_usd = script_dir / "urdf" / "lekiwi" / "lekiwi.usd"

    urdf_path = args.urdf if args.urdf else str(default_urdf)
    usd_path = args.usd if args.usd else str(default_usd)

    # Check URDF exists
    if not Path(urdf_path).exists():
        print(f"Error: URDF file not found: {urdf_path}")
        return

    convert_urdf_to_usd(urdf_path, usd_path, args.fix_base)


if __name__ == "__main__":
    main()
    simulation_app.close()
