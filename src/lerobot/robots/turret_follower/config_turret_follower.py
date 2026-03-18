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

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("turretF")
@dataclass
class TurretFollowerConfig(RobotConfig):
    port: str
    baudrate: int = 4_000_000
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None
    use_degrees: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    """
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "topCam": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=1920,
                height=1080,
            ),
            "wristCam": OpenCVCameraConfig(
                index_or_path=2,
                fps=30,
                width=1920,
                height=1080,
            )
        }
    )
    """