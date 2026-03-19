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
from functools import cached_property

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode

from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_turret_follower import TurretFollowerConfig

logger = logging.getLogger(__name__)


class TurretFollower(Robot):
    config_class = TurretFollowerConfig
    name = "turretF"

    def __init__(self, config: TurretFollowerConfig):
        super().__init__(config)
        self.config = config
        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "shoulder": Motor(1, "xm430-w350", norm_mode),
                "gripper": Motor(2, "xm430-w350", norm_mode),
            },
            calibration=self.calibration,
        )
        self.bus.default_baudrate = self.config.baudrate
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motor_obs_ft(self) -> dict[str, type]:
        return {
            "shoulder.pos": float,
            "gripper.pos": float,
            "shoulder.current": float,
            "gripper.current": float,
        }

    @property
    def _camera_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motor_obs_ft, **self._camera_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {"shoulder.pos": float, "gripper.pos": float}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()
        for cam in self.cameras.values():
            cam.connect()
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
            for motor in self.bus.motors:
                # Keep non-gripper joints in Position mode.
                if motor == "gripper":
                    # Gripper uses Current-based Position mode with a lower
                    # current limit for safer compliant grasping.
                    self.bus.write("Operating_Mode", motor, OperatingMode.CURRENT_POSITION.value)
                    self.bus.write("Current_Limit", motor, 70)
                else:
                    # Position mode: single-turn PID position tracking.
                    # Present_Current reflects motor load and is fed back to the leader
                    # as Goal_Current to produce proportional haptic resistance.
                    self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                    self.bus.write("Current_Limit", motor, 1000)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        pos = self.bus.sync_read("Present_Position")
        cur = self.bus.sync_read("Present_Current")
        obs = {
            "shoulder.pos": float(pos["shoulder"]),
            "gripper.pos": float(pos["gripper"]),
            "shoulder.current": float(cur["shoulder"]),
            "gripper.current": float(cur["gripper"]),
        }

        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")
        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        self.bus.sync_write("Goal_Position", goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
