# Data Collection Guide

This guide covers the complete workflow for collecting teleoperation datasets with LeKiwi, converting them to LeRobot format, and uploading to HuggingFace Hub.

## Table of Contents

- [Recording Demonstrations](#recording-demonstrations)
- [HDF5 Dataset Structure](#hdf5-dataset-structure)
- [Episode Management](#episode-management)
- [Converting to LeRobot Format](#converting-to-lerobot-format)
- [Uploading to HuggingFace Hub](#uploading-to-huggingface-hub)
- [Best Practices](#best-practices)
- [Replaying Demonstrations](#replaying-demonstrations)
- [Data Statistics](#data-statistics)
- [Troubleshooting](#troubleshooting)

## Recording Demonstrations

### Basic Recording

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda \
    --enable_cameras \
    --record \
    --dataset_file=./datasets/lekiwi_demos.hdf5
```

### Recording with Episode Limit

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda \
    --enable_cameras \
    --record \
    --dataset_file=./datasets/lekiwi_demos.hdf5 \
    --num_demos=50  # Stop after 50 successful demos
```

### Using Helper Script

```bash
# Basic recording
./lekisaac/playground/run_lekiwi_teleop.sh --record

# With episode limit
./lekisaac/playground/run_lekiwi_teleop.sh --record --num-demos 50

# Custom output file
./lekisaac/playground/run_lekiwi_teleop.sh --record --dataset-file ./my_dataset.hdf5
```

## HDF5 Dataset Structure

Recorded datasets use the IsaacLab HDF5 format:

```
dataset.hdf5
└── data/
    ├── demo_0/
    │   ├── actions              # (T, 9) [arm(5)+gripper(1)+base_velocity(3)]
    │   ├── obs/
    │   │   ├── joint_pos        # (T, N) joint positions
    │   │   ├── joint_vel        # (T, N) joint velocities
    │   │   ├── robot_pose       # (T, 3) base position [x, y, z]
    │   │   ├── robot_orientation # (T, 4) base quaternion [w, x, y, z]
    │   │   ├── wrist             # (T, H, W, 3) wrist camera RGB
    │   │   └── base              # (T, H, W, 3) base camera RGB
    │   └── attrs: {num_samples, seed, success}
    ├── demo_1/
    │   └── ...
    └── demo_N/
        └── ...
```

## Episode Management

### Marking Episodes

During teleoperation:
- Press **'B'** to start teleoperation
- Press **'N'** after successful task completion (marks as success)
- Press **'R'** if the attempt failed (marks as failed)

**Important**: Only episodes marked with 'N' (success=True) are used for training.

### Episode Attributes

Each episode stores:
- `num_samples`: Number of timesteps
- `seed`: Random seed used
- `success`: Whether the episode was successful

## Converting to LeRobot Format

### Prerequisites

LeRobot must be installed in a **separate conda environment** (recommended):

```bash
# Create LeRobot environment
conda create -n lerobot python=3.10
conda activate lerobot

# Install LeRobot v0.3.3 (tested version)
pip install lerobot==0.3.3
```

### Conversion Command

```bash
# Activate LeRobot environment
conda activate lerobot

# Navigate to leisaac directory
cd /path/to/leisaac

# Convert HDF5 to LeRobot format
python scripts/convert/isaaclab2lerobot.py \
    --dataset_path_or_repo your_username/lekiwi_dataset \
    --robot_type so101_follower \
    --fps 30 \
    --hdf5_files ./datasets/lekiwi_demos.hdf5 \
    --task "Pick up the cube and place it on the target"
```

### Conversion Options

| Option | Description | Default |
|--------|-------------|---------|
| `--dataset_path_or_repo` | HuggingFace repo ID (e.g., `username/dataset_name`) | Required |
| `--robot_type` | `so101_follower` or `bi_so101_follower` | `so101_follower` |
| `--fps` | Frames per second | `30` |
| `--hdf5_files` | HDF5 file(s) to convert (space-separated) | Required |
| `--task` | Task description for language-conditioned policies | Required |
| `--push_to_hub` | Upload to HuggingFace Hub after conversion | `False` |

### Multiple HDF5 Files

You can combine multiple recording sessions:

```bash
python scripts/convert/isaaclab2lerobot.py \
    --dataset_path_or_repo your_username/lekiwi_combined \
    --robot_type so101_follower \
    --fps 30 \
    --hdf5_files \
        ./datasets/session1.hdf5 \
        ./datasets/session2.hdf5 \
        ./datasets/session3.hdf5 \
    --task "Mobile manipulation task"
```

## Uploading to HuggingFace Hub

### Step 1: HuggingFace Authentication

```bash
# Install huggingface_hub if not already installed
pip install huggingface_hub

# Login to HuggingFace (requires access token)
huggingface-cli login
```

To get an access token:
1. Go to https://huggingface.co/settings/tokens
2. Create a new token with "Write" permissions
3. Paste the token when prompted

### Step 2: Convert and Upload

Add `--push_to_hub` flag to automatically upload after conversion:

```bash
python scripts/convert/isaaclab2lerobot.py \
    --dataset_path_or_repo your_username/lekiwi_pick_cube \
    --robot_type so101_follower \
    --fps 30 \
    --hdf5_files ./datasets/lekiwi_demos.hdf5 \
    --task "Pick up the cube and place it on the target" \
    --push_to_hub
```

### Step 3: Verify Upload

After upload, your dataset will be available at:
```
https://huggingface.co/datasets/your_username/lekiwi_pick_cube
```

### Manual Upload (Alternative)

If you converted without `--push_to_hub`, you can upload manually:

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset("your_username/lekiwi_pick_cube")
dataset.push_to_hub()
```

## Best Practices

### Before Recording

1. **Test the environment**: Run without `--record` first
2. **Check camera views**: Ensure cameras capture relevant information
3. **Calibrate**: Run with `--recalibrate` if SO101Leader needs calibration
4. **Plan tasks**: Know exactly what demonstrations you need

### During Recording

1. **Start clean**: Press 'B' to begin, reset with 'R' before each demonstration
2. **Be consistent**: Perform tasks in a similar manner
3. **Natural pace**: Don't rush - natural movements train better
4. **Mark correctly**: Only mark truly successful attempts with 'N'

### After Recording

1. **Verify data**: Check the HDF5 file was created
2. **Review episodes**: Optionally replay to verify quality
3. **Backup data**: Copy to a safe location before conversion
4. **Document**: Note task descriptions, settings used

### Task Descriptions

Write clear, concise task descriptions for language-conditioned policies:

**Good examples:**
- "Pick up the red cube and place it on the blue plate"
- "Open the drawer and retrieve the object inside"
- "Navigate to the table and grasp the bottle"

**Avoid:**
- "Do the task" (too vague)
- "Pick up the cube that is located at coordinates (0.5, 0.3, 0.1) and move it to..." (too specific)

## Replaying Demonstrations

Replay recorded demonstrations to verify quality:

```bash
# Using leisaac replay script
python scripts/environments/teleoperation/replay.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --num_envs=1 \
    --device=cuda \
    --enable_cameras \
    --replay_mode=action \
    --dataset_file=./datasets/lekiwi_demos.hdf5 \
    --select_episodes 0 1 2  # Replay specific episodes
```

## Data Statistics

Check dataset statistics with Python:

```python
import h5py

with h5py.File("./datasets/lekiwi_demos.hdf5", "r") as f:
    data = f["data"]
    num_demos = len(data.keys())

    successful = 0
    failed = 0
    total_steps = 0

    for demo_name in data.keys():
        demo = data[demo_name]
        steps = demo.attrs.get("num_samples", 0)
        total_steps += steps

        if demo.attrs.get("success", False):
            successful += 1
        else:
            failed += 1

    print(f"Total demonstrations: {num_demos}")
    print(f"Successful: {successful}")
    print(f"Failed: {failed}")
    print(f"Total timesteps: {total_steps}")
    print(f"Average steps per episode: {total_steps / num_demos:.1f}")
```

## Troubleshooting

### Recording Issues

**Recording Not Starting**
1. Check `--record` flag is present
2. Verify output directory exists and is writable
3. Ensure `--enable_cameras` is set if you need camera data

**Data File Empty or Corrupted**
1. Don't interrupt recording abruptly (use Ctrl+C for graceful exit)
2. Check disk space
3. Verify file permissions

**Episodes Not Marked as Successful**
1. Press 'N' (not 'R') for successful episodes
2. Ensure you press the key before environment auto-resets
3. Check terminal for confirmation messages

### Conversion Issues

**ModuleNotFoundError: No module named 'lerobot'**
- Ensure you're in the LeRobot conda environment: `conda activate lerobot`

**KeyError during conversion**
- HDF5 structure may not match expected format
- Check that cameras were enabled during recording
- Verify observation keys match expected names (front, wrist)

**Version compatibility issues**
- Use LeRobot v0.3.3 (tested version)
- Newer versions may have API changes

### Upload Issues

**Authentication failed**
1. Run `huggingface-cli login` again
2. Ensure token has "Write" permissions
3. Check internet connection

**Repository already exists**
- Use a different repo name, or
- Delete the existing repo on HuggingFace first

## Complete Workflow Example

Here's a complete example from recording to HuggingFace upload:

```bash
# 1. Activate IsaacLab environment and record
conda activate isaaclab
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda \
    --enable_cameras \
    --record \
    --dataset_file=./datasets/pick_cube_demos.hdf5 \
    --num_demos=50

# 2. Switch to LeRobot environment and convert + upload
conda activate lerobot
python scripts/convert/isaaclab2lerobot.py \
    --dataset_path_or_repo myusername/lekiwi_pick_cube_v1 \
    --robot_type so101_follower \
    --fps 30 \
    --hdf5_files ./datasets/pick_cube_demos.hdf5 \
    --task "Pick up the cube and place it on the target plate" \
    --push_to_hub

# 3. Dataset is now available at:
# https://huggingface.co/datasets/myusername/lekiwi_pick_cube_v1
```

## Next Steps

After uploading your dataset:

1. **Train policies**: Use LeRobot, GR00T N1.5, or other frameworks
2. **Fine-tune VLAs**: SmolVLA, OpenVLA, etc.
3. **Deploy**: Test trained policies in simulation or on real hardware

See the [leisaac documentation](https://github.com/LightwheelAI/leisaac) for policy training guides.
