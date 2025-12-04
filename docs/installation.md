# Installation Guide

This guide covers the installation process for lekisaac.

## Prerequisites

### 1. IsaacSim and IsaacLab

Follow the [IsaacLab installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html) first.

**Recommended versions:**
- IsaacSim 5.0+
- IsaacLab 2.2.0+
- Python 3.11+

```bash
# Create conda environment
conda create -n lekisaac python=3.11
conda activate lekisaac

# Install IsaacSim
pip install 'isaacsim[all,extscache]==5.0.0' --extra-index-url https://pypi.nvidia.com

# Clone and install IsaacLab
git clone git@github.com:isaac-sim/IsaacLab.git
cd IsaacLab
git checkout v2.2.0
./isaaclab.sh --install
```

### 2. Install leisaac

```bash
# Clone leisaac
git clone https://github.com/LightwheelAI/leisaac.git
cd leisaac

# Install leisaac
pip install -e source/leisaac
```

## Installing lekisaac

### Option 1: As a Git Submodule (Recommended)

```bash
cd leisaac
git submodule add https://github.com/vpraise00/lekisaac.git lekisaac
pip install -e lekisaac/source/lekisaac
```

### Option 2: Standalone Installation

```bash
git clone https://github.com/vpraise00/lekisaac.git
cd lekisaac
pip install -e source/lekisaac
```

## Hardware Setup

### SO101Leader Configuration

1. **Connect SO101Leader via USB**

2. **Find the USB port**:
   ```bash
   # List all connected serial ports
   ls /dev/ttyACM* /dev/ttyUSB*

   # Check recently connected device (run right after plugging in)
   dmesg | tail -20

   # List USB devices with details
   lsusb

   # More detailed port information
   ls -la /dev/serial/by-id/
   ```

   일반적으로 SO101Leader는 `/dev/ttyACM0` 또는 `/dev/ttyACM1`로 인식됩니다.
   여러 장치가 연결된 경우 `dmesg` 출력에서 마지막으로 연결된 장치를 확인하세요.

3. **Set up port permissions**:
   ```bash
   sudo chmod 666 /dev/ttyACM0
   # Or add user to dialout group (permanent)
   sudo usermod -aG dialout $USER
   # Log out and back in for group change to take effect
   ```

4. **Calibrate the leader device** (first time only):
   ```bash
   python lekisaac/scripts/teleop_lekiwi.py \
       --task=LeKisaac-LeKiwi-Teleop-v0 \
       --port=/dev/ttyACM0 \
       --recalibrate
   ```

## Verification

Run a quick test to verify installation:

```bash
# Test without hardware (keyboard-only would require modification)
python -c "import lekisaac; print('lekisaac installed successfully!')"

# Test with IsaacSim (requires display)
python lekisaac/scripts/teleop_lekiwi.py \
    --task=LeKisaac-LeKiwi-Teleop-v0 \
    --device=cuda \
    --headless
```

## Troubleshooting

### Port Permission Denied

```
PermissionError: [Errno 13] Permission denied: '/dev/ttyACM0'
```

Solution:
```bash
sudo chmod 666 /dev/ttyACM0
```

### SO101Leader Not Found

Check USB connection and verify port:
```bash
# Check available ports
ls /dev/ttyACM* /dev/ttyUSB*

# Check kernel messages for USB connection
dmesg | grep -i tty

# Check if device is recognized
lsusb | grep -i serial
```

포트가 보이지 않으면:
- USB 케이블 연결 상태 확인
- 다른 USB 포트에 연결 시도
- USB 케이블 교체 (데이터 케이블인지 확인)

### Import Errors

Ensure leisaac is installed and in your Python path:
```bash
pip install -e /path/to/leisaac/source/leisaac
pip install -e /path/to/lekisaac/source/lekisaac
```

## Next Steps

- [LeKiwi Robot Setup](lekiwi_robot.md)
- [Teleoperation Guide](teleoperation.md)
- [Data Collection](data_collection.md)
