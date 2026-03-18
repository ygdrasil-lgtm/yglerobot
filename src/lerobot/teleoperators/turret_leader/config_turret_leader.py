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
    # Maximum absolute current (mA) allowed for leader haptic feedback.
    # Used both for motor Current_Limit and software clipping of Goal_Current.
    max_current: int = 100
    # Backward-compatible alias; if provided, it overrides max_current.
    current_limit: int | None = None
    # Per-joint scaling applied to follower Present_Current before writing to
    # leader Goal_Current. Tune to adjust haptic intensity per joint.
    # Formula: Goal_Current = clamp(-follower_current * gain * direction, -max_current, max_current)
    feedback_gain: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder": 0.2,
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
