# Available Tasks

This document lists all available task environments in LeKisaac.

## Task Overview

| Task ID | Description | Scene |
|---------|-------------|-------|
| `LeKisaac-LeKiwi-Teleop-v0` | Basic teleoperation environment | Ground plane + test cube |
| `LeKisaac-LeKiwi-Kitchen-v0` | Kitchen environment for manipulation | Kitchen scene with furniture |
| `LeKisaac-LeKiwi-ToolAugmented-v0` | Elevated arm (0.8m taller) for high surface tasks | Kitchen + elevated cube |

---

## LeKisaac-LeKiwi-Teleop-v0

Basic teleoperation environment with a simple ground plane and a small cube for grasping tests.

### Scene Contents
- Ground plane with high friction
- 2cm red cube for grasping practice
- Dome lighting

### Quick Start

```bash
# Basic teleoperation
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda

# With recording
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --record \
    --dataset_file=./datasets/teleop_demos.hdf5 \
    --num_demos=50
```

---

## LeKisaac-LeKiwi-Kitchen-v0

Kitchen environment for realistic mobile manipulation tasks.

### Scene Contents
- Kitchen scene (`kiwi_kitchen.usd`) - visual environment with furniture and appliances
- Kiwi fruit (`Kiwi.usd`) - graspable object with physics enabled
- Bowl (`Bowl.usd`) - target container with physics enabled
- Dome lighting

### Quick Start

```bash
# Basic teleoperation in kitchen
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Kitchen-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda

# With recording
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Kitchen-v0 \
    --port=/dev/ttyACM0 \
    --record \
    --dataset_file=./datasets/kitchen_demos.hdf5 \
    --num_demos=50

# High quality rendering
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Kitchen-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda \
    --quality
```

---

## LeKisaac-LeKiwi-ToolAugmented-v0

Environment with an elevated LeKiwi robot (0.8m taller arm) for tasks on high surfaces like tables and counters.

### Robot Configuration
- Uses `lekiwi_augmented.usd` with modified URDF
- Arm is 0.8m higher than standard LeKiwi
- Same base and wheel configuration as standard

### Scene Contents
- Kitchen scene (`kiwi_kitchen.usd`) - visual environment
- Green cube at 1m height for elevated manipulation practice
- Dome lighting

### Quick Start

```bash
# First, convert the augmented URDF to USD (one-time)
python lekisaac/scripts/convert_urdf_to_usd.py --augmented-only

# Basic teleoperation
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-ToolAugmented-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda

# With recording
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-ToolAugmented-v0 \
    --port=/dev/ttyACM0 \
    --record \
    --dataset_file=./datasets/tool_augmented_demos.hdf5 \
    --num_demos=50
```

---

## Common Options

All tasks support the following command-line options:

| Option | Default | Description |
|--------|---------|-------------|
| `--task` | `LeKisaac-LeKiwi-Teleop-v0` | Task environment ID |
| `--port` | `/dev/ttyACM0` | SO101Leader serial port |
| `--device` | `cuda` | Compute device (cuda/cpu) |
| `--num_envs` | `1` | Number of parallel environments |
| `--record` | `False` | Enable HDF5 recording |
| `--dataset_file` | `./datasets/lekiwi_dataset.hdf5` | Output dataset path |
| `--num_demos` | `0` | Number of demos to record (0=infinite) |
| `--resume` | `False` | Resume recording in existing file |
| `--recalibrate` | `False` | Recalibrate SO101Leader |
| `--quality` | `False` | Enable high quality rendering |
| `--base_linear_speed` | `0.3` | Base linear velocity (m/s) |
| `--base_angular_speed` | `0.5` | Base angular velocity (rad/s) |
| `--step_hz` | `30` | Environment step rate (Hz) |

## Controls

| Control | Key | Description |
|---------|-----|-------------|
| Arm | SO101-Leader | Move leader device to control arm |
| Forward | W | Move base forward |
| Backward | S | Move base backward |
| Strafe Left | A | Move base left |
| Strafe Right | D | Move base right |
| Rotate Left | Z | Rotate base counter-clockwise |
| Rotate Right | X | Rotate base clockwise |
| Start | B | Begin teleoperation |
| Reset (failed) | R | Reset environment (mark as failed) |
| Reset (success) | N | Reset environment (mark as successful) |

## Adding New Tasks

To add a new task with a custom scene:

1. Place your USD scene in `lekisaac/assets/scenes/`
2. Create a new task directory in `source/lekisaac/lekisaac/tasks/`
3. Create environment config inheriting scene setup from existing tasks
4. Register with gymnasium in `__init__.py`
5. Import in `tasks/__init__.py`

See `lekiwi_kitchen/` for an example implementation.
