#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
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
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_turret_leader import TurretLeaderConfig

logger = logging.getLogger(__name__)


class TurretLeader(Teleoperator):
    config_class = TurretLeaderConfig
    name = "turretL"

    def __init__(self, config: TurretLeaderConfig):
        super().__init__(config)
        self.config = config
        self.feedback_motor_map = {
            "shoulder": "shoulder",
            "gripper": "gripper",
        }
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "shoulder": Motor(1, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(2, "xl330-m077", MotorNormMode.RANGE_M100_100),
            },
            calibration=self.calibration,
        )
        self.bus.default_baudrate = self.config.baudrate

    @property
    def action_features(self) -> dict[str, type]:
        return {"shoulder.pos": float, "gripper.pos": float}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {f"{name}.current": float for name in self.feedback_motor_map}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @property
    def _max_feedback_current(self) -> int:
        # `current_limit` is kept as a backward-compatible alias for `max_current`.
        if self.config.current_limit is not None:
            return int(self.config.current_limit)
        return int(self.config.max_current)

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()
        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        self.bus.disable_torque()
        if self.calibration:
            user_input = input(
                f"Press ENTER to use existing calibration for '{self.id}', "
                "or type 'c' and press ENTER to re-run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing existing calibration for '{self.id}' to the motors.")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        # Use EXTENDED_POSITION during calibration so motors can rotate freely.
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        print(
            "Move all joints sequentially through their entire ranges of motion.\n"
            "Recording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(list(self.bus.motors))

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            max_current = self._max_feedback_current
            for motor in self.bus.motors:
                # Pure Current Control mode (0): motor torque ∝ Goal_Current directly.
                # Present_Position is still readable in this mode.
                # The operator's hand motion is tracked via Present_Position and sent
                # to the follower as Goal_Position. Haptic resistance is achieved by
                # writing scaled follower Present_Current back as Goal_Current here.
                self.bus.write("Operating_Mode", motor, OperatingMode.CURRENT.value)
                self.bus.write("Current_Limit", motor, max_current)
                self.bus.write("Goal_Current", motor, 0)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        action = self.bus.sync_read("Present_Position")
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return {
            "shoulder.pos": float(action["shoulder"]),
            "gripper.pos": float(action["gripper"]),
        }

    def send_feedback(self, feedback: dict[str, float]) -> None:
        """Map follower Present_Current → leader Goal_Current for haptic resistance.

        Follower operates in Extended Position mode; its load current is proportional
        to external forces. To render resistance, we apply opposite torque on the
        leader: Goal_Current ∝ -(follower_current). The command is scaled by
        per-joint gain and clamped to ±max_current for safety.
        """
        limit = self._max_feedback_current
        gains = self.config.feedback_gain
        directions = self.config.feedback_direction
        for follower_joint, leader_joint in self.feedback_motor_map.items():
            key = f"{follower_joint}.current"
            if key not in feedback:
                continue
            gain = gains.get(follower_joint, 1.0)
            direction = directions.get(follower_joint, 1.0)
            target_current = int(max(-limit, min(limit, -feedback[key] * gain * direction)))
            self.bus.write("Goal_Current", leader_joint, target_current, normalize=False)

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
