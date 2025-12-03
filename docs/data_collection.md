# Data Collection Guide

This guide covers how to collect teleoperation datasets with LeKiwi.

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
    --record \
    --dataset_file=./datasets/lekiwi_demos.hdf5 \
    --num_demos=50  # Stop after 50 successful demos
```

### Resume Recording

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --record \
    --dataset_file=./datasets/lekiwi_demos.hdf5 \
    --resume  # Continue from existing file
```

## HDF5 Dataset Structure

Recorded datasets use the IsaacLab HDF5 format:

```
dataset.hdf5
└── data/
    ├── demo_0/
    │   ├── actions              # (T, 9) [arm(5)+gripper(1)+wheels(3)]
    │   ├── obs/
    │   │   ├── joint_pos        # (T, N) joint positions
    │   │   ├── joint_vel        # (T, N) joint velocities
    │   │   ├── robot_pose       # (T, 3) base position [x, y, z]
    │   │   ├── robot_orientation # (T, 4) base quaternion [w, x, y, z]
    │   │   ├── wrist             # (T, H, W, 3) wrist camera RGB
    │   │   └── top               # (T, H, W, 3) top camera RGB
    │   └── attrs: {num_samples, seed, success}
    ├── demo_1/
    │   └── ...
    └── demo_N/
        └── ...
```

## Episode Management

### Marking Episodes

During teleoperation:
- Press **'N'** after successful task completion
- Press **'R'** if the attempt failed

Only episodes marked with 'N' (success=True) are used for training.

### Episode Attributes

Each episode stores:
- `num_samples`: Number of timesteps
- `seed`: Random seed used
- `success`: Whether the episode was successful

## Converting to LeRobot Format

To use the data with LeRobot policies, convert the HDF5:

```bash
# Activate LeRobot environment (separate conda env recommended)
conda activate lerobot

# Convert to LeRobot format
python scripts/convert/isaaclab2lerobot.py \
    --dataset_path_or_repo your_username/lekiwi_dataset \
    --robot_type so101_follower \
    --fps 30 \
    --hdf5_files ./datasets/lekiwi_demos.hdf5 \
    --task "Mobile manipulation task description" \
    --push_to_hub  # Optional: upload to HuggingFace
```

## Best Practices

### Before Recording

1. **Test the environment**: Run without `--record` first
2. **Check camera views**: Ensure cameras capture relevant information
3. **Calibrate**: Recalibrate SO101Leader if needed
4. **Plan tasks**: Know exactly what demonstrations you need

### During Recording

1. **Start clean**: Reset before each demonstration
2. **Be consistent**: Perform tasks in a similar manner
3. **Natural pace**: Don't rush - natural movements train better
4. **Mark correctly**: Only mark truly successful attempts with 'N'

### After Recording

1. **Verify data**: Check the HDF5 file was created
2. **Review episodes**: Optionally replay to verify quality
3. **Backup data**: Copy to a safe location
4. **Document**: Note task descriptions, settings used

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
    total_steps = 0

    for demo_name in data.keys():
        demo = data[demo_name]
        if demo.attrs.get("success", False):
            successful += 1
        total_steps += demo.attrs.get("num_samples", 0)

    print(f"Total demonstrations: {num_demos}")
    print(f"Successful demonstrations: {successful}")
    print(f"Total timesteps: {total_steps}")
```

## Troubleshooting

### Recording Not Starting

1. Check `--record` flag is present
2. Verify output directory is writable
3. Ensure file doesn't exist (or use `--resume`)

### Data File Empty or Corrupted

1. Don't interrupt recording abruptly
2. Let the script exit gracefully (Ctrl+C)
3. Check disk space

### Episodes Not Marked as Successful

1. Press 'N' (not 'R') for successful episodes
2. Ensure you press the key before environment auto-resets
3. Check terminal for confirmation messages

## Next Steps

After collecting data:
1. Convert to LeRobot format for policy training
2. Train policies (GR00T N1.5, SmolVLA, etc.)
3. Deploy trained policies in simulation or on real hardware
