# LeKisaac

LeKiwi robot integration with NVIDIA IsaacLab simulation via [leisaac](https://github.com/LightwheelAI/leisaac).

This package enables teleoperation data collection for the LeKiwi mobile manipulator, combining SO101Leader arm control with keyboard-based omnidirectional base movement.

## Features

- **Hybrid Teleoperation**: SO101Leader for arm + keyboard (WASD/ZX) for base
- **Mobile Manipulation**: Full support for omnidirectional base movement
- **Data Collection**: Record demonstrations in HDF5 format
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
# or
sudo usermod -aG dialout $USER
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
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda \
    --enable_cameras
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

### Dataset Collection Guide

For detailed instructions on collecting demonstration datasets for imitation learning, refer to the official [IsaacLab Teleoperation and Imitation Learning Guide](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html).

This guide covers:
- **Recording demonstrations** via teleoperation with various input devices
- **Replaying collected data** for verification and debugging
- **Managing HDF5 datasets** for training imitation learning policies
- **Automated data augmentation** using Isaac Lab Mimic to generate large synthetic datasets from a small number of human demonstrations

## Documentation

- [Installation Guide](docs/installation.md)
- [LeKiwi Robot Setup](docs/lekiwi_robot.md)
- [Teleoperation Guide](docs/teleoperation.md)
- [Data Collection](docs/data_collection.md)

## Project Structure

```
lekisaac/
├── source/lekisaac/lekisaac/   # Python package
│   ├── assets/                  # Robot configurations
│   ├── devices/                 # Teleoperation devices
│   │   ├── lekiwi_device.py    # Hybrid SO101+keyboard device
│   │   └── action_process.py   # Action processing
│   ├── tasks/                   # Task environments
│   │   └── lekiwi_teleop/      # Teleoperation task
│   └── utils/                   # Utilities
├── scripts/                     # Runnable scripts
│   └── teleop_lekiwi.py        # Main teleoperation script
├── urdf/lekiwi/                # LeKiwi URDF and meshes
├── playground/                  # Example scripts
│   └── run_lekiwi_teleop.sh    # Quick start script
└── docs/                        # Documentation
```

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
