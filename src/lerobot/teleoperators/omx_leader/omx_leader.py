#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import (
    DriveMode,
    DynamixelMotorsBus,
    OperatingMode,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_omx_leader import OmxLeaderConfig

logger = logging.getLogger(__name__)


class OmxLeader(Teleoperator):
    """
    - [OMX](https://github.com/ROBOTIS-GIT/open_manipulator),
        expansion, developed by Woojin Wie and Junha Cha from [ROBOTIS](https://ai.robotis.com/)
    """

    config_class = OmxLeaderConfig
    name = "omx_leader"

    def __init__(self, config: OmxLeaderConfig):
        super().__init__(config)
        self.config = config
        # Map follower joint names to leader joint names for haptic current feedback.
        self.feedback_motor_map = {
            "shoulder_pan": "shoulder_pan",
            "shoulder_lift": "shoulder_lift",
            "elbow_flex": "elbow_flex",
            "wrist_flex": "wrist_flex",
            "wrist_roll": "wrist_roll",
            "gripper": "gripper",
        }
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "xl330-m077", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.bus.default_baudrate = self.config.baudrate

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {f"{follower_joint}.current": float for follower_joint in self.feedback_motor_map}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        self.bus.disable_torque()
        logger.info(f"\nUsing factory default calibration values for {self}")
        logger.info(f"\nWriting default configuration of {self} to the motors")
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.CURRENT_POSITION.value)

        for motor in self.bus.motors:
            if motor == "gripper":
                self.bus.write("Drive_Mode", motor, DriveMode.INVERTED.value)
            else:
                self.bus.write("Drive_Mode", motor, DriveMode.NON_INVERTED.value)
        drive_modes = {motor: 1 if motor == "gripper" else 0 for motor in self.bus.motors}

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=drive_modes[motor],
                homing_offset=0 if motor != "gripper" else 100,
                range_min=0,
                range_max=4095,
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.CURRENT_POSITION.value)

            if motor == "gripper":
                self.bus.write("Drive_Mode", motor, DriveMode.INVERTED.value)
            else:
                self.bus.write("Drive_Mode", motor, DriveMode.NON_INVERTED.value)

        # Current-position mode plus goal-current enables simple current-based haptic behavior.
        self.bus.write("Current_Limit", "gripper", 100)
        self.bus.write("Goal_Current", "gripper", 100)
        self.bus.write("Homing_Offset", "gripper", 100)
        # Set gripper's goal pos in current position mode so that we can use it as a trigger.
        self.bus.enable_torque("gripper")
        if self.is_calibrated:
            self.bus.write("Goal_Position", "gripper", self.config.gripper_open_pos)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        action = self.bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        for follower_joint, leader_joint in self.feedback_motor_map.items():
            key = f"{follower_joint}.current"
            if key not in feedback:
                continue

            # Map observed follower current magnitude to leader goal current.
            target_current = int(max(0, min(100, abs(feedback[key]))))
            self.bus.write("Goal_Current", leader_joint, target_current, normalize=False)

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
