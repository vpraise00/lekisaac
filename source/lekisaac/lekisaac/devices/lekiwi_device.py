# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

"""Hybrid teleoperation device for LeKiwi robot.

Combines SO101Leader for arm control with keyboard for base control.

Key bindings for base control:
    W: Move forward
    S: Move backward
    A: Strafe left
    D: Strafe right
    Z: Rotate left (counter-clockwise)
    X: Rotate right (clockwise)

Arm control:
    Move SO101Leader device to control the arm
"""

import os
import json
import weakref
import numpy as np
from collections.abc import Callable
from typing import Dict, Tuple

import carb
import omni

from leisaac.devices.device_base import Device
from leisaac.devices.lerobot.common.motors import (
    FeetechMotorsBus,
    Motor,
    MotorNormMode,
    MotorCalibration,
    OperatingMode,
)
from leisaac.devices.lerobot.common.errors import (
    DeviceAlreadyConnectedError,
    DeviceNotConnectedError,
)
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_MOTOR_LIMITS


class LeKiwiDevice(Device):
    """A hybrid teleoperation device for LeKiwi mobile manipulator.

    Combines SO101Leader hardware for arm teleoperation with keyboard
    control for omnidirectional base movement.
    """

    def __init__(
        self,
        env,
        port: str = "/dev/ttyACM0",
        recalibrate: bool = False,
        calibration_file_name: str = "lekiwi_leader.json",
        base_linear_speed: float = 0.3,
        base_angular_speed: float = 0.5,
    ):
        """Initialize the LeKiwi teleoperation device.

        Args:
            env: The IsaacLab environment.
            port: Serial port for SO101Leader arm device.
            recalibrate: Whether to recalibrate the SO101Leader.
            calibration_file_name: Name of the calibration file.
            base_linear_speed: Linear velocity for base movement (m/s).
            base_angular_speed: Angular velocity for base rotation (rad/s).
        """
        super().__init__(env)
        self.port = port
        self.base_linear_speed = base_linear_speed
        self.base_angular_speed = base_angular_speed

        # Initialize SO101Leader for arm control
        self.calibration_path = os.path.join(
            os.path.dirname(__file__), ".cache", calibration_file_name
        )
        if not os.path.exists(self.calibration_path) or recalibrate:
            self.calibrate()
        calibration = self._load_calibration()

        self._bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=calibration,
        )
        self._motor_limits = SO101_FOLLOWER_MOTOR_LIMITS

        # Connect SO101Leader
        self.connect()

        # Initialize keyboard for base control
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(
                event, *args
            ),
        )

        # Base velocity command buffer [linear_x, linear_y, angular_z]
        self._base_velocity = np.zeros(3)

        # Create key bindings for base control
        self._create_base_key_bindings()

        # Flags and callbacks
        self._started = False
        self._reset_state = False
        self._additional_callbacks = {}

        self._display_controls()

    def __del__(self):
        """Release resources."""
        self.stop_keyboard_listener()

    def stop_keyboard_listener(self):
        """Stop the keyboard event listener."""
        if (
            hasattr(self, "_input")
            and hasattr(self, "_keyboard")
            and hasattr(self, "_keyboard_sub")
        ):
            self._input.unsubscribe_to_keyboard_events(
                self._keyboard, self._keyboard_sub
            )
            self._keyboard_sub = None

    def __str__(self) -> str:
        """Return string representation."""
        msg = "LeKiwi Device for mobile manipulation teleoperation.\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tArm Control: Move SO101-Leader device\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tBase Control:\n"
        msg += "\t  W: Move forward\n"
        msg += "\t  S: Move backward\n"
        msg += "\t  A: Strafe left\n"
        msg += "\t  D: Strafe right\n"
        msg += "\t  Z: Rotate left (CCW)\n"
        msg += "\t  X: Rotate right (CW)\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tB: Start control\n"
        msg += "\tR: Reset (mark as failed)\n"
        msg += "\tN: Reset (mark as successful)\n"
        msg += "\tCtrl+C: Quit\n"
        return msg

    def _display_controls(self):
        """Pretty print the control scheme."""

        def print_command(char, info):
            char += " " * (30 - len(char))
            print("{}\t{}".format(char, info))

        print("")
        print("=" * 60)
        print("LeKiwi Teleoperation Device")
        print("=" * 60)
        print("\n[Arm Control - SO101 Leader]")
        print_command("Move SO101-Leader", "Control arm joints")
        print("\n[Base Control - Keyboard]")
        print_command("W / S", "Move forward / backward")
        print_command("A / D", "Strafe left / right")
        print_command("Z / X", "Rotate left / right")
        print("\n[System Controls]")
        print_command("B", "Start teleoperation")
        print_command("R", "Reset (mark as failed)")
        print_command("N", "Reset (mark as successful)")
        print_command("Ctrl+C", "Quit")
        print("=" * 60)
        print("")

    def _create_base_key_bindings(self):
        """Create key bindings for base velocity control."""
        self._BASE_KEY_MAPPING = {
            # [linear_x, linear_y, angular_z]
            "W": np.array([self.base_linear_speed, 0.0, 0.0]),   # Forward
            "S": np.array([-self.base_linear_speed, 0.0, 0.0]),  # Backward
            "A": np.array([0.0, self.base_linear_speed, 0.0]),   # Strafe left
            "D": np.array([0.0, -self.base_linear_speed, 0.0]),  # Strafe right
            "Z": np.array([0.0, 0.0, self.base_angular_speed]),  # Rotate left (CCW)
            "X": np.array([0.0, 0.0, -self.base_angular_speed]), # Rotate right (CW)
        }

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events for base control and system commands."""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            key_name = event.input.name
            # Base movement keys
            if key_name in self._BASE_KEY_MAPPING:
                self._base_velocity += self._BASE_KEY_MAPPING[key_name]
            # System control keys
            elif key_name == "B":
                self._started = True
                self._reset_state = False
            elif key_name == "R":
                self._started = False
                self._reset_state = True
                if "R" in self._additional_callbacks:
                    self._additional_callbacks["R"]()
            elif key_name == "N":
                self._started = False
                self._reset_state = True
                if "N" in self._additional_callbacks:
                    self._additional_callbacks["N"]()

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            key_name = event.input.name
            # Release base movement keys
            if key_name in self._BASE_KEY_MAPPING:
                self._base_velocity -= self._BASE_KEY_MAPPING[key_name]

        return True

    def get_device_state(self):
        """Get the current state from SO101Leader."""
        return self._bus.sync_read("Present_Position")

    def get_base_velocity(self):
        """Get the current base velocity command."""
        return self._base_velocity.copy()

    def input2action(self):
        """Convert device input to action dictionary."""
        state = {}
        reset = state["reset"] = self._reset_state
        state["started"] = self._started

        if reset:
            self._reset_state = False
            return state

        # Get arm state from SO101Leader
        state["joint_state"] = self.get_device_state()
        # Get base velocity from keyboard
        state["base_velocity"] = self.get_base_velocity()

        ac_dict = {}
        ac_dict["reset"] = reset
        ac_dict["started"] = self._started
        ac_dict["lekiwi_device"] = True  # Flag for action processing

        if reset:
            return ac_dict

        ac_dict["joint_state"] = state["joint_state"]
        ac_dict["base_velocity"] = state["base_velocity"]
        ac_dict["motor_limits"] = self._motor_limits

        return ac_dict

    def reset(self):
        """Reset device state."""
        self._base_velocity = np.zeros(3)

    def add_callback(self, key: str, func: Callable):
        """Add callback for a keyboard key."""
        self._additional_callbacks[key] = func

    @property
    def started(self) -> bool:
        return self._started

    @property
    def reset_state(self) -> bool:
        return self._reset_state

    @reset_state.setter
    def reset_state(self, value: bool):
        self._reset_state = value

    @property
    def motor_limits(self) -> Dict[str, Tuple[float, float]]:
        return self._motor_limits

    @property
    def is_connected(self) -> bool:
        return self._bus.is_connected

    def disconnect(self):
        """Disconnect SO101Leader."""
        if not self.is_connected:
            raise DeviceNotConnectedError("SO101-Leader is not connected.")
        self._bus.disconnect()
        print("LeKiwi device disconnected.")

    def connect(self):
        """Connect SO101Leader."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError("SO101-Leader is already connected.")
        self._bus.connect()
        self.configure()
        print("LeKiwi device connected.")

    def configure(self) -> None:
        """Configure SO101Leader motors."""
        self._bus.disable_torque()
        self._bus.configure_motors()
        for motor in self._bus.motors:
            self._bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def calibrate(self):
        """Calibrate SO101Leader."""
        self._bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
        )
        self.connect()

        print("\nRunning calibration of LeKiwi arm (SO101-Leader)")
        self._bus.disable_torque()
        for motor in self._bus.motors:
            self._bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(
            "Move SO101-Leader to the middle of its range of motion and press ENTER..."
        )
        homing_offset = self._bus.set_half_turn_homings()
        print("Move all joints sequentially through their entire ranges of motion.")
        print("Recording positions. Press ENTER to stop...")
        range_mins, range_maxes = self._bus.record_ranges_of_motion()

        calibration = {}
        for motor, m in self._bus.motors.items():
            calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offset[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
        self._bus.write_calibration(calibration)
        self._save_calibration(calibration)
        print(f"Calibration saved to {self.calibration_path}")

        self.disconnect()

    def _load_calibration(self) -> Dict[str, MotorCalibration]:
        """Load calibration from file."""
        with open(self.calibration_path, "r") as f:
            json_data = json.load(f)
        calibration = {}
        for motor_name, motor_data in json_data.items():
            calibration[motor_name] = MotorCalibration(
                id=int(motor_data["id"]),
                drive_mode=int(motor_data["drive_mode"]),
                homing_offset=int(motor_data["homing_offset"]),
                range_min=int(motor_data["range_min"]),
                range_max=int(motor_data["range_max"]),
            )
        return calibration

    def _save_calibration(self, calibration: Dict[str, MotorCalibration]):
        """Save calibration to file."""
        save_calibration = {
            k: {
                "id": v.id,
                "drive_mode": v.drive_mode,
                "homing_offset": v.homing_offset,
                "range_min": v.range_min,
                "range_max": v.range_max,
            }
            for k, v in calibration.items()
        }
        if not os.path.exists(os.path.dirname(self.calibration_path)):
            os.makedirs(os.path.dirname(self.calibration_path))
        with open(self.calibration_path, "w") as f:
            json.dump(save_calibration, f, indent=4)
