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

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("turretL")
@dataclass
class TurretLeaderConfig(TeleoperatorConfig):
    port: str
    baudrate: int = 4_000_000
    # Maximum absolute haptic command current in mA.
    # Software clipping is applied in mA, then converted to each leader motor's
    # raw current unit (LSB-dependent) before writing Goal_Current.
    max_current: int = 100
    # Backward-compatible alias; if provided, it overrides max_current.
    current_limit: int | None = None
    # Optional per-joint max current in mA for leader haptic output.
    # If a joint is missing, it falls back to current_limit/max_current.
    max_current_per_joint: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 20.0*2.69,
            "gripper": 1750.0,
        }
    )
    # Per-joint scaling applied to follower Present_Current before writing to
    # leader Goal_Current. Tune to adjust haptic intensity per joint.
    # Formula: Goal_Current = clamp(-follower_current * gain * direction, -max_current, max_current)
    feedback_gain: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 0.2,
            "gripper": 0.9,
        }
    )
    # Follower current conversion from raw register unit to mA.
    # XM430-W350: ~2.69 mA/LSB. XL330-M077: approximately ~1 mA/LSB (supply-current based).
    feedback_input_lsb_ma: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 2.69,
            "gripper": 2.69,
        }
    )
    # Follower current full-scale for normalization in mA.
    # Normalized effort = clip(input_mA / feedback_input_max_ma, -1, 1).
    # follower의 current가 feedback_input_max_ma보다 크면, leader에 최대 haptic feedback이 전달된다. follower의 current가 feedback_input_max_ma보다 작으면, leader에 전달되는 haptic feedback이 줄어든다.
    # follower의 current_limit 확인 필요. 예시: XM430-W350의 경우, current_limit이 1000mA라면 feedback_input_max_ma를 1000mA로 설정하여, follower의 최대 current에 대응하는 최대 haptic feedback이 leader에 전달되도록 할 수 있다.
    feedback_input_max_ma: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 1000.0,
            "gripper": 70.0,
        }
    )
    # If True, treat the follower current as supply current (e.g. XL330) and
    # apply low-pass filtering to reduce ripple/noise mismatch vs phase current.
    feedback_input_is_supply_current: dict[str, bool] = field(
        default_factory=lambda: {
            "shoulder": False,
            "gripper": True,
        }
    )
    # Low-pass filter alpha for supply-current compensation.
    # filtered = alpha * new + (1 - alpha) * prev
    # alpha가 작을수록 더 부드럽고 지연이 큼
    # alpha가 클수록 반응이 빠르지만 노이즈 전달이 큼
    # 채터링이 심하면: alpha를 더 낮춤 (예: 0.10)
    # 반응이 너무 느리면: alpha를 높임 (예: 0.25~0.35)
    feedback_input_filter_alpha: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 0.35,
            "gripper": 0.118,
        }
    )
    # Deadband in mA to suppress tiny current noise around zero.
    feedback_deadband_ma: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 10.0,
            "gripper": 10.0,
        }
    )
    # Leader command conversion from mA to raw Goal_Current unit.
    # XM430-W350: ~2.69 mA/LSB. XL330-M077: ~1 mA/LSB.
    leader_goal_current_lsb_ma: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 2.69,
            "gripper": 1.0,
        }
    )
    # Per-joint direction correction. Keep +1.0 unless a joint is mechanically inverted.
    feedback_direction: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 1.0,
            "gripper": 1.0,
        }
    )
    # Enables one-line haptic debug output per feedback cycle.
    # Prints follower Present_Current and applied leader Goal_Current together.
    debug_feedback: bool = False
    # If True, prints feedback lines to stdout with flush=True every cycle so
    # values are visible in real time during `lerobot-teleoperate` execution.
    debug_feedback_realtime: bool = False
    # Optional realtime print rate limit in Hz.
    # - `None` or <= 0: print every feedback cycle
    # - positive value: print at most this many lines per second (e.g. 10.0)
    debug_feedback_realtime_hz: float | None = 1.0

    # Gripper feedback mode selection.
    # "proportional": Current haptic feedback. Resistance proportional to follower current.
    # "threshold_constant": Threshold-based constant force. Detects object contact (threshold)
    #   and applies constant resistance ONLY when current exceeds threshold, to prevent chattering.
    gripper_feedback_mode: str = "threshold_constant"

    # Backward-compatible single threshold in mA for threshold_constant mode.
    # If `gripper_feedback_threshold_on_ma` is None, this value is used as the ON threshold.
    # follower 로봇의 gripper의 curent_based position 모드에서 current_limit과 동일한 또는 그보다 약간 낮은 값을 적용한다.
    # 예시: XM430-W350의 경우, current_limit이 1000mA라면 threshold를 700mA로 설정하여, 그립핑이 감지되도록 할 수 있다.
    gripper_feedback_threshold_ma: float = 65.0

    # Hysteresis ON threshold in mA for threshold_constant mode.
    # Contact becomes active when |follower_current| >= threshold_on.
    # If None, falls back to `gripper_feedback_threshold_ma`.
    gripper_feedback_threshold_on_ma: float | None = None

    # Hysteresis OFF threshold in mA for threshold_constant mode.
    # Contact is released when |follower_current| <= threshold_off.
    # Use a smaller value than ON threshold to reduce boundary chattering.
    gripper_feedback_threshold_off_ma: float = 50.0

    # Constant resistance force in mA applied when follower current exceeds threshold
    # in threshold_constant mode. Sign matches follower current direction.
    gripper_feedback_constant_force_ma: float = 700.0
