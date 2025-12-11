# LeKisaac

<p align="center">
  <img src="docs/images/lekisaac_banner.png" alt="LeKisaac Banner" width="800">
</p>

LeKiwi robot integration with NVIDIA IsaacLab simulation via [leisaac](https://github.com/LightwheelAI/leisaac).

This package enables teleoperation data collection for the LeKiwi mobile manipulator, combining SO101Leader arm control with keyboard-based omnidirectional base movement.

## Features

- **Hybrid Teleoperation**: SO101Leader for arm + keyboard (WASD/ZX) for base
- **Mobile Manipulation**: Velocity-based holonomic control for smooth omnidirectional movement
- **Data Collection**: Record demonstrations in HDF5 format
- **Multiple Environments**: Basic teleop, kitchen scene, tool-augmented tasks
- **IsaacLab Integration**: Built on leisaac for seamless simulation

## Quick Start

### Prerequisites

1. Install [IsaacLab](https://isaac-sim.github.io/IsaacLab/) (v2.2.0+ for IsaacSim 5.0)
2. Install [leisaac](https://github.com/LightwheelAI/leisaac)
3. SO101Leader hardware device

### Installation

```bash
# Clone lekisaac (if not already a submodule)
cd leisaac
git submodule add https://github.com/vpraise00/lekisaac.git lekisaac

# Install lekisaac package
pip install -e lekisaac/source/lekisaac
```

### USB Port Permission

```bash
sudo chmod 666 /dev/ttyACM0
# or permanently:
sudo usermod -aG dialout $USER
```

### URDF to USD Conversion (Required First Time)

```bash
# Convert all robot variants
python lekisaac/scripts/convert_urdf_to_usd.py

# Or convert specific variant only
python lekisaac/scripts/convert_urdf_to_usd.py --standard-only
python lekisaac/scripts/convert_urdf_to_usd.py --augmented-only
```

### Running Teleoperation

```bash
# Basic teleoperation
./lekisaac/playground/run_lekiwi_teleop.sh

# With recording
./lekisaac/playground/run_lekiwi_teleop.sh --record --num-demos 10

# Specify port
./lekisaac/playground/run_lekiwi_teleop.sh --port /dev/ttyACM1
```

Or run directly:

```bash
# Basic teleop
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda

# Kitchen environment
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Kitchen-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda

# With recording
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --record \
    --dataset_file=./datasets/lekiwi_demos.hdf5 \
    --num_demos=50
```

### Replay Demonstrations

```bash
python lekisaac/scripts/replay_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --device=cuda \
    --dataset_file=./datasets/lekiwi_demos.hdf5 \
    --select_episodes 0 1 2
```

## Controls

| Control | Key(s) | Description |
|---------|--------|-------------|
| Arm | SO101-Leader | Move leader device to control arm |
| Forward | W | Move base forward |
| Backward | S | Move base backward |
| Strafe Left | A | Move base left |
| Strafe Right | D | Move base right |
| Rotate Left | Z | Rotate base counter-clockwise |
| Rotate Right | X | Rotate base clockwise |
| Start | B | Begin teleoperation |
| Reset (failed) | R | Reset environment (mark episode as failed) |
| Reset (success) | N | Reset environment (mark episode as successful) |

## Available Tasks

| Task ID | Description | Scene |
|---------|-------------|-------|
| `LeKisaac-LeKiwi-Teleop-v0` | Basic teleoperation | Ground plane + test cube |
| `LeKisaac-LeKiwi-Kitchen-v0` | Kitchen manipulation | Kitchen scene with cube/bowl |
| `LeKisaac-LeKiwi-ToolAugmented-v0` | Elevated arm (0.8m taller) | Kitchen scene with pan on counter |

## Robot Variants

| Variant | Config | Description |
|---------|--------|-------------|
| `lekiwi.usd` | `LEKIWI_CFG` | Standard simulation |
| `lekiwi_augmented.usd` | `LEKIWI_AUGMENTED_CFG` | 0.8m taller arm for elevated tasks |

## Project Structure

```
lekisaac/
├── source/lekisaac/lekisaac/   # Python package
│   ├── assets/                  # Robot configurations (LEKIWI_CFG, etc.)
│   ├── devices/                 # Teleoperation devices
│   │   ├── lekiwi_device.py    # Hybrid SO101Leader + keyboard device
│   │   └── action_process.py   # Arm/base action processing
│   ├── envs/actions/           # Custom action terms
│   │   └── holonomic_base_velocity_action.py
│   ├── tasks/                   # Task environments
│   │   ├── lekiwi_teleop/      # LeKisaac-LeKiwi-Teleop-v0
│   │   ├── lekiwi_kitchen/     # LeKisaac-LeKiwi-Kitchen-v0
│   │   └── lekiwi_tool_augmented/  # LeKisaac-LeKiwi-ToolAugmented-v0
│   └── utils/                   # Utilities
├── scripts/
│   ├── teleop_lekiwi.py        # Main teleoperation script
│   ├── replay_lekiwi.py        # Replay recorded demonstrations
│   └── convert_urdf_to_usd.py  # URDF to USD converter
├── urdf/lekiwi/                # LeKiwi URDF variants and meshes
├── assets/
│   ├── scenes/                 # Environment USD files
│   └── objects/                # Graspable object USD files
├── playground/                  # Quick start scripts
│   └── run_lekiwi_teleop.sh
└── docs/                        # Documentation
```

## Documentation

- [Installation Guide](docs/installation.md) - Setting up lekisaac with IsaacLab and leisaac
- [LeKiwi Robot Setup](docs/lekiwi_robot.md) - Hardware configuration and URDF details
- [Teleoperation Guide](docs/teleoperation.md) - Controls and teleoperation workflow
- [Data Collection & HuggingFace Upload](docs/data_collection.md) - Recording demos, converting to LeRobot format, uploading to HuggingFace Hub
- [Available Tasks](docs/available_tasks.md) - List of registered task environments

### Dataset Collection Guide

For detailed instructions on collecting demonstration datasets for imitation learning, refer to the official [IsaacLab Teleoperation and Imitation Learning Guide](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html).

## Requirements

- Python 3.11+
- IsaacSim 5.0+
- IsaacLab 2.2.0+
- leisaac
- SO101Leader hardware

## License

Apache-2.0

## Acknowledgements

Built on [leisaac](https://github.com/LightwheelAI/leisaac) and [IsaacLab](https://github.com/isaac-sim/IsaacLab).
