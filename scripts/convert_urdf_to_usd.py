#!/usr/bin/env python3
# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Convert LeKiwi URDF files to USD format.

This script converts both the standard and augmented LeKiwi URDF files to USD.

Usage:
    python lekisaac/scripts/convert_urdf_to_usd.py [--augmented-only]
"""

import argparse
from pathlib import Path

# Must import isaacsim before any other isaaclab imports
import isaacsim  # noqa: F401
from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg


def convert_urdf(urdf_path: Path, usd_path: Path) -> None:
    """Convert a single URDF file to USD.

    Args:
        urdf_path: Path to the input URDF file.
        usd_path: Path to the output USD file.
    """
    print(f"Converting: {urdf_path} -> {usd_path}")

    cfg = UrdfConverterCfg(
        asset_path=str(urdf_path),
        usd_dir=str(usd_path.parent),
        usd_file_name=usd_path.name,
        fix_base=False,  # Mobile base - not fixed
        make_instanceable=False,
        merge_fixed_joints=False,  # Keep all joints for proper articulation
        self_collision=False,
    )

    converter = UrdfConverter(cfg)
    print(f"Successfully converted to: {usd_path}")
    return converter


def main():
    parser = argparse.ArgumentParser(description="Convert LeKiwi URDF to USD")
    parser.add_argument(
        "--augmented-only",
        action="store_true",
        help="Only convert the augmented URDF",
    )
    parser.add_argument(
        "--standard-only",
        action="store_true",
        help="Only convert the standard URDF",
    )
    args = parser.parse_args()

    # Get URDF directory
    script_dir = Path(__file__).parent.resolve()
    urdf_dir = script_dir.parent / "urdf" / "lekiwi"

    # Define URDF/USD pairs
    conversions = []

    if not args.augmented_only:
        conversions.append({
            "urdf": urdf_dir / "lekiwi.urdf",
            "usd": urdf_dir / "lekiwi.usd",
            "name": "LeKiwi (standard)",
        })

    if not args.standard_only:
        conversions.append({
            "urdf": urdf_dir / "lekiwi_augmented.urdf",
            "usd": urdf_dir / "lekiwi_augmented.usd",
            "name": "LeKiwi (augmented, 0.8m taller)",
        })

    # Convert each URDF
    for conv in conversions:
        if not conv["urdf"].exists():
            print(f"Warning: {conv['urdf']} does not exist, skipping {conv['name']}")
            continue

        print(f"\n{'='*60}")
        print(f"Converting {conv['name']}")
        print(f"{'='*60}")
        convert_urdf(conv["urdf"], conv["usd"])

    print("\nConversion complete!")


if __name__ == "__main__":
    main()
