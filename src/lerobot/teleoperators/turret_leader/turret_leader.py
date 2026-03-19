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
import os
import sys
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
        self._last_realtime_debug_print_s = 0.0
        self._filtered_input_current_ma: dict[str, float] = {}
        self._gripper_contact_active = False
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
    def _max_feedback_current(self) -> float:
        # `current_limit` is kept as a backward-compatible alias for `max_current`.
        if self.config.current_limit is not None:
            return float(self.config.current_limit)
        return float(self.config.max_current)

    def _max_feedback_current_for_joint(self, joint: str) -> float:
        if joint in self.config.max_current_per_joint:
            return float(self.config.max_current_per_joint[joint])
        return self._max_feedback_current

    def _apply_deadband(self, value: float, deadband: float) -> float:
        if abs(value) <= deadband:
            return 0.0
        return value - deadband if value > 0 else value + deadband

    def _to_leader_raw_current(self, leader_joint: str, current_ma: float) -> int:
        lsb = self.config.leader_goal_current_lsb_ma.get(leader_joint, 2.69)
        if lsb <= 0:
            lsb = 1.0
        return int(round(current_ma / lsb))

    def _get_gripper_hysteresis_thresholds(self) -> tuple[float, float]:
        threshold_on = self.config.gripper_feedback_threshold_on_ma
        if threshold_on is None:
            threshold_on = self.config.gripper_feedback_threshold_ma
        threshold_on = abs(float(threshold_on))

        threshold_off = abs(float(self.config.gripper_feedback_threshold_off_ma))
        if threshold_off > threshold_on:
            threshold_off = threshold_on
        return threshold_on, threshold_off

    @property
    def _realtime_debug_enabled(self) -> bool:
        # Env fallback is useful when CLI nested config parsing is uncertain.
        env_force = os.getenv("LEROBOT_DEBUG_FEEDBACK_REALTIME", "").strip().lower()
        env_true = env_force in {"1", "true", "yes", "on"}

        # CLI fallback for cases where nested teleop config flags are not
        # propagated into the dataclass instance.
        cli_true = False
        for arg in sys.argv[1:]:
            if arg.startswith("--teleop.debug_feedback_realtime="):
                value = arg.split("=", 1)[1].strip().lower()
                cli_true = value in {"1", "true", "yes", "on"}
                break
            if arg.startswith("--debug_feedback_realtime="):
                value = arg.split("=", 1)[1].strip().lower()
                cli_true = value in {"1", "true", "yes", "on"}
                break

        return bool(self.config.debug_feedback_realtime or env_true or cli_true)

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()
        self.configure()
        logger.info(
            "TurretLeader feedback debug | "
            f"debug_feedback={self.config.debug_feedback} "
            f"debug_feedback_realtime={self.config.debug_feedback_realtime} "
            f"effective_realtime={self._realtime_debug_enabled} "
            f"debug_feedback_realtime_hz={self.config.debug_feedback_realtime_hz}"
        )
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
            for motor in self.bus.motors:
                # Pure Current Control mode (0): motor torque ∝ Goal_Current directly.
                # Present_Position is still readable in this mode.
                # The operator's hand motion is tracked via Present_Position and sent
                # to the follower as Goal_Position. Haptic resistance is achieved by
                # writing scaled follower Present_Current back as Goal_Current here.
                self.bus.write("Operating_Mode", motor, OperatingMode.CURRENT.value)
                max_current_ma = self._max_feedback_current_for_joint(motor)
                max_current_raw = max(0, self._to_leader_raw_current(motor, max_current_ma))
                self.bus.write("Current_Limit", motor, max_current_raw)
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
        gains = self.config.feedback_gain
        directions = self.config.feedback_direction
        debug_parts: list[str] = []
        for follower_joint, leader_joint in self.feedback_motor_map.items():
            limit_ma = self._max_feedback_current_for_joint(leader_joint)
            key = f"{follower_joint}.current"
            if key not in feedback:
                continue

            # 1) Convert follower raw current register value to mA.
            input_lsb_ma = self.config.feedback_input_lsb_ma.get(follower_joint, 2.69)
            follower_current_raw = float(feedback[key])
            follower_current_ma = follower_current_raw * input_lsb_ma

            # 2) Compensate supply-current measurements (XL330-like behavior) via LPF.
            if self.config.feedback_input_is_supply_current.get(follower_joint, False):
                alpha = self.config.feedback_input_filter_alpha.get(follower_joint, 0.15)
                alpha = max(0.0, min(1.0, alpha))
                prev = self._filtered_input_current_ma.get(follower_joint, follower_current_ma)
                follower_current_ma = alpha * follower_current_ma + (1.0 - alpha) * prev
                self._filtered_input_current_ma[follower_joint] = follower_current_ma

            # 3) Apply deadband to suppress tiny near-zero noise.
            deadband_ma = self.config.feedback_deadband_ma.get(follower_joint, 0.0)
            follower_current_ma = self._apply_deadband(follower_current_ma, deadband_ma)

            # 4) Normalize follower effort to [-1, 1] and re-scale to leader max mA.
            input_max_ma = self.config.feedback_input_max_ma.get(follower_joint, limit_ma)
            input_max_ma = max(1e-6, input_max_ma)
            normalized_effort = max(-1.0, min(1.0, follower_current_ma / input_max_ma))

            gain = gains.get(follower_joint, 1.0)
            direction = directions.get(follower_joint, 1.0)
            target_current_ma = max(
                -limit_ma,
                min(limit_ma, -normalized_effort * limit_ma * gain * direction),
            )

            # Check if gripper needs special feedback mode handling.
            if follower_joint == "gripper" and self.config.gripper_feedback_mode == "threshold_constant":
                # Hysteresis-based threshold mode for gripper:
                # - Activate when |I| >= threshold_on
                # - Keep active until |I| <= threshold_off
                # - Active state outputs constant opposite-direction resistance
                threshold_on_ma, threshold_off_ma = self._get_gripper_hysteresis_thresholds()
                constant_force_ma = abs(self.config.gripper_feedback_constant_force_ma)
                follower_abs = abs(follower_current_ma)

                if self._gripper_contact_active:
                    if follower_abs <= threshold_off_ma:
                        self._gripper_contact_active = False
                elif follower_abs >= threshold_on_ma:
                    self._gripper_contact_active = True

                if not self._gripper_contact_active:
                    target_current_ma = 0.0
                else:
                    # Apply opposite-direction constant force for resistance.
                    sign = 1.0 if follower_current_ma >= 0 else -1.0
                    target_current_ma = -sign * constant_force_ma * direction
            else:
                if follower_joint == "gripper":
                    self._gripper_contact_active = False
                # Proportional mode (default for all joints including gripper)
                target_current_ma = max(
                    -limit_ma,
                    min(limit_ma, -normalized_effort * limit_ma * gain * direction),
                )
            # 5) Quantize mA command to leader model raw current unit.
            target_current_raw = self._to_leader_raw_current(leader_joint, target_current_ma)
            max_raw = max(0, self._to_leader_raw_current(leader_joint, limit_ma))
            target_current_raw = int(max(-max_raw, min(max_raw, target_current_raw)))

            self.bus.write("Goal_Current", leader_joint, target_current_raw, normalize=False)
            debug_parts.append(
                f"{follower_joint}: Iraw={follower_current_raw:.1f}, I={follower_current_ma:.1f}mA"
                f" -> {leader_joint}.Goal_Current={target_current_raw} ({target_current_ma:.1f}mA)"
            )

        if debug_parts:
            debug_line = "Haptic feedback | " + " | ".join(debug_parts)
            if self.config.debug_feedback:
                logger.info(debug_line)
            if self._realtime_debug_enabled:
                hz = self.config.debug_feedback_realtime_hz
                if hz is None or hz <= 0:
                    print(debug_line, flush=True)
                else:
                    now_s = time.perf_counter()
                    period_s = 1.0 / hz
                    if now_s - self._last_realtime_debug_print_s >= period_s:
                        print(debug_line, flush=True)
                        self._last_realtime_debug_print_s = now_s

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
