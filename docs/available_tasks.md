# Available Tasks

This document lists all available task environments in LeKisaac.

## Task Overview

| Task ID | Description | Scene |
|---------|-------------|-------|
| `LeKisaac-LeKiwi-Teleop-v0` | Basic teleoperation environment | Ground plane + test cube |
| `LeKisaac-LeKiwi-Kitchen-v0` | Kitchen environment for manipulation | Kitchen scene with furniture |
| `LeKisaac-LeKiwi-PickMoveSpatula-v0` | Pick and move spatula (0.8m taller arm) | Kitchen + spatula on counter |
| `LeKisaac-LeKiwi-FlipEgg-v0` | Flip egg with spatula (0.8m taller arm) | Kitchen + frying pan, egg, spatula |
| `LeKisaac-LeKiwi-CutCube-v0` | Cut cube with knife (0.8m taller arm) | Kitchen + cutting board, knife, soft cube |

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

## LeKisaac-LeKiwi-PickMoveSpatula-v0

Environment for picking up and moving a spatula from the kitchen counter. Uses an elevated LeKiwi robot (0.8m taller arm) for reaching high surfaces.

### Robot Configuration
- Uses `lekiwi_elevated_smt.usd` with modified URDF
- Arm is 0.8m higher than standard LeKiwi
- Enlarged center standoff for stability
- Same base and wheel configuration as standard

### Scene Contents
- Kitchen scene (`kiwi_kitchen.usd`) - visual environment with collision
- Spatula (`Spatula.usd`) - graspable tool on kitchen counter
- Dome lighting

### Quick Start

```bash
# First, convert the augmented URDF to USD (one-time)
python lekisaac/scripts/convert_urdf_to_usd.py --augmented-only

# Basic teleoperation
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-PickMoveSpatula-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda

# With recording
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-PickMoveSpatula-v0 \
    --port=/dev/ttyACM0 \
    --record \
    --dataset_file=./datasets/pick_move_spatula_demos.hdf5 \
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

---

## LeKisaac-LeKiwi-FlipEgg-v0

Environment for flipping an egg on a frying pan using a spatula. Uses an elevated LeKiwi robot (0.8m taller arm).

### Scene Contents
- Kitchen scene (`kiwi_kitchen.usd`)
- Frying pan on gas range (kinematic)
- Egg on the frying pan (dynamic)
- Spatula for flipping

### Quick Start

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-FlipEgg-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda \
    --enable_cameras
```

---

## LeKisaac-LeKiwi-CutCube-v0

Environment for cutting a soft cube with a knife on a cutting board. Uses an elevated LeKiwi robot (0.8m taller arm). When the knife presses down on the cube, the cube splits into two halves.

### Scene Contents
- Kitchen scene (`kiwi_kitchen.usd`)
- Cutting board on counter (kinematic)
- Soft pink cube on cutting board (dynamic)
- Knife for cutting

### Cutting Mechanic
- Position knife directly above the cube
- Press knife down into the cube
- When knife penetrates enough, cube splits into two vertical halves

### Quick Start

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-CutCube-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda \
    --enable_cameras
```
