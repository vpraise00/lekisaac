# Teleoperation Guide

This guide covers how to use the LeKiwi teleoperation system.

## Overview

LeKiwi uses a hybrid teleoperation approach:
- **Arm Control**: SO101Leader hardware device (6 DOF)
- **Base Control**: Keyboard (WASD for movement, ZX for rotation)

## Running Teleoperation

### Using the Shell Script

The easiest way to start teleoperation:

```bash
# Basic teleoperation
./lekisaac/playground/run_lekiwi_teleop.sh

# With custom port
./lekisaac/playground/run_lekiwi_teleop.sh --port /dev/ttyACM1

# With recording enabled
./lekisaac/playground/run_lekiwi_teleop.sh --record

# High quality rendering
./lekisaac/playground/run_lekiwi_teleop.sh --quality
```

### Using Python Directly

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --device=cuda \
    --num_envs=1 \
    --enable_cameras
```

## Control Scheme

### Arm Control (SO101Leader)

Move the SO101Leader device physically to control the robot arm:

| Leader Motion | Robot Action |
|--------------|--------------|
| Pan (rotate base) | shoulder_pan joint |
| Lift (raise arm) | shoulder_lift joint |
| Flex elbow | elbow_flex joint |
| Flex wrist | wrist_flex joint |
| Roll wrist | wrist_roll joint |
| Open/close gripper | gripper joint |

### Base Control (Keyboard)

| Key | Action |
|-----|--------|
| W | Move forward |
| S | Move backward |
| A | Strafe left |
| D | Strafe right |
| Z | Rotate counter-clockwise |
| X | Rotate clockwise |

### System Controls

| Key | Action |
|-----|--------|
| B | Start teleoperation |
| R | Reset environment (mark episode as failed) |
| N | Reset environment (mark episode as successful) |
| Ctrl+C | Quit |

## Velocity Settings

Adjust base movement speed:

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --base_linear_speed=0.5 \  # m/s (default: 0.3)
    --base_angular_speed=1.0   # rad/s (default: 0.5)
```

## Step Rate

Control the simulation stepping rate:

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --step_hz=30  # Hz (default: 30)
```

Higher values = more responsive but more CPU usage.

## Workflow

### Basic Teleoperation

1. **Start the script**: Run teleoperation command
2. **Wait for initialization**: IsaacSim will load (may take 30-60 seconds)
3. **Press 'B'**: Begin teleoperation
4. **Control the robot**:
   - Move SO101Leader for arm
   - Use WASD/ZX for base
5. **Reset as needed**:
   - Press 'R' to reset (failed attempt)
   - Press 'N' to reset (successful attempt)
6. **Exit**: Press Ctrl+C

### Data Collection Workflow

1. **Plan your demonstrations**: Know what task you want to record
2. **Start with recording**:
   ```bash
   ./lekisaac/playground/run_lekiwi_teleop.sh --record --num-demos 10
   ```
3. **Press 'B'** to start
4. **Perform task**: Control the robot to complete the task
5. **Mark success**: Press 'N' when task is complete
6. **Repeat**: Continue until desired number of demos
7. **Data saved**: Check the output HDF5 file

## Troubleshooting

### Robot Not Responding to Arm Commands

1. Check SO101Leader USB connection
2. Verify port is correct (`--port`)
3. Try recalibration: `--recalibrate`

### Base Movement Too Fast/Slow

Adjust velocity parameters:
```bash
--base_linear_speed=0.2  # Slower linear
--base_angular_speed=0.3  # Slower rotation
```

### Lag or Stuttering

1. Reduce step rate: `--step_hz=20`
2. Disable cameras: Remove `--enable_cameras`
3. Use headless mode for testing: `--headless`

### Cannot Start Teleoperation

1. Make sure 'B' key is pressed
2. Check for error messages in terminal
3. Verify environment loaded correctly

## Tips

1. **Practice first**: Run without recording to get comfortable
2. **Start slow**: Use lower velocity settings initially
3. **Use viewpoints**: Move the IsaacSim camera to see the task clearly
4. **Reset often**: Don't try to save failed attempts
5. **Quality matters**: Better demonstrations = better trained policies

## Next Steps

- [Data Collection Guide](data_collection.md)
