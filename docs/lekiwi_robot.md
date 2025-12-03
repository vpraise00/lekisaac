# LeKiwi Robot Setup

LeKiwi is a mobile manipulator combining:
- 3 omnidirectional wheels for holonomic base movement
- SO-101 compatible arm with 6 DOF (5 arm joints + gripper)

## Robot Specifications

### Base Configuration

| Component | Specification |
|-----------|--------------|
| Drive Type | 3-wheel omnidirectional |
| Wheel Arrangement | Y-configuration (120° apart) |
| Wheel Type | 4" Omni-directional wheel |
| Motors | ST3215 Servo Motors |

### Arm Configuration

| Joint | Range (degrees) | Motor |
|-------|----------------|-------|
| shoulder_pan | -110° to +110° | STS3215 |
| shoulder_lift | -100° to +100° | STS3215 |
| elbow_flex | -100° to +90° | STS3215 |
| wrist_flex | -95° to +95° | STS3215 |
| wrist_roll | -160° to +160° | STS3215 |
| gripper | -10° to +100° | STS3215 |

## URDF to USD Conversion

The LeKiwi URDF must be converted to USD format for IsaacLab.

### Step 1: Prepare URDF

The URDF is located at `lekisaac/urdf/lekiwi/lekiwi.urdf`.

Update mesh file paths if needed:
```xml
<mesh filename="file:///path/to/lekisaac/urdf/lekiwi/meshes/..." />
```

### Step 2: Convert to USD

Use IsaacSim's URDF importer:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()

import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = True
import_config.fix_base = False  # Mobile base
import_config.import_inertia_tensor = True
import_config.self_collision = True

result = urdf_interface.parse_urdf(
    "/path/to/lekisaac/urdf/lekiwi/lekiwi.urdf",
    import_config
)

urdf_interface.import_robot(
    "/path/to/output/lekiwi.usd",
    result,
    import_config,
    "lekiwi"
)

simulation_app.close()
```

### Step 3: Update Asset Configuration

After conversion, update the USD path in `lekisaac/source/lekisaac/lekisaac/assets/lekiwi.py`:

```python
LEKIWI_ASSET_PATH = Path("/path/to/lekiwi.usd")
```

## Joint Name Mapping

The LeKiwi robot uses specific joint names that must be mapped correctly:

### Wheel Joints (Continuous)
- `ST3215_Servo_Motor_v1_2_Revolute_60` → `wheel_left`
- `ST3215_Servo_Motor_v1_1_Revolute_62` → `wheel_right`
- `ST3215_Servo_Motor_v1_Revolute_64` → `wheel_back`

### Arm Joints (Limited)
- `STS3215_03a_v1_Revolute_45` → `shoulder_pan`
- `STS3215_03a_v1_1_Revolute_49` → `shoulder_lift`
- `STS3215_03a_v1_2_Revolute_51` → `elbow_flex`
- `STS3215_03a_v1_3_Revolute_53` → `wrist_flex`
- `STS3215_03a_Wrist_Roll_v1_Revolute_55` → `wrist_roll`
- `STS3215_03a_v1_4_Revolute_57` → `gripper`

**Note**: You may need to rename joints in the USD file or update the configuration to match your URDF export.

## Calibration

### SO101Leader Calibration

First-time calibration is required to sync the leader and follower:

```bash
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --port=/dev/ttyACM0 \
    --recalibrate
```

Follow the prompts:
1. Move the leader to the middle of its range
2. Press ENTER
3. Move all joints through their full range
4. Press ENTER to save calibration

Calibration is saved to:
`lekisaac/source/lekisaac/lekisaac/devices/.cache/lekiwi_leader.json`

## Troubleshooting

### Arm Not Moving

1. Check SO101Leader connection
2. Verify calibration file exists
3. Re-run calibration with `--recalibrate`

### Base Movement Incorrect

1. Verify wheel joint names match configuration
2. Check inverse kinematics parameters in `action_process.py`
3. Adjust wheel radius and base radius if needed

### Joint Limits Exceeded

Update joint limits in `lekisaac/source/lekisaac/lekisaac/assets/lekiwi.py` to match your specific hardware.
