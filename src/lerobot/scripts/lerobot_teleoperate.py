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

"""
Simple script to control a robot from teleoperation.

Example:

```shell
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --robot.id=black \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue \
    --display_data=true
```

Example teleoperation with bimanual so100:

```shell
lerobot-teleoperate \
  --robot.type=bi_so_follower \
  --robot.left_arm_config.port=/dev/tty.usbmodem5A460822851 \
  --robot.right_arm_config.port=/dev/tty.usbmodem5A460814411 \
  --robot.id=bimanual_follower \
  --robot.left_arm_config.cameras='{
    wrist: {"type": "opencv", "index_or_path": 1, "width": 640, "height": 480, "fps": 30},
  }' --robot.right_arm_config.cameras='{
    wrist: {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30},
  }' \
  --teleop.type=bi_so_leader \
  --teleop.left_arm_config.port=/dev/tty.usbmodem5A460852721 \
  --teleop.right_arm_config.port=/dev/tty.usbmodem5A460819811 \
  --teleop.id=bimanual_leader \
  --display_data=true
```

"""

import logging
import time
from dataclasses import asdict, dataclass
from numbers import Real
from pprint import pformat

import rerun as rr

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.cameras.zmq.configuration_zmq import ZMQCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_openarm_follower,
    bi_so_follower,
    dynamixel_follower,
    earthrover_mini_plus,
    hope_jr,
    koch_follower,
    make_robot_from_config,
    omx_follower,
    openarm_follower,
    reachy2,
    so_follower,
    turret_follower,
    unitree_g1 as unitree_g1_robot,
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_openarm_leader,
    bi_so_leader,
    gamepad,
    homunculus,
    keyboard,
    koch_leader,
    make_teleoperator_from_config,
    omx_leader,
    openarm_leader,
    openarm_mini,
    reachy2_teleoperator,
    so_leader,
    turret_leader,
    unitree_g1,
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


@dataclass
class TeleoperateConfig:
    # TODO: pepijn, steven: if more robots require multiple teleoperators (like lekiwi) its good to make this possibele in teleop.py and record.py with List[Teleoperator]
    teleop: TeleoperatorConfig
    robot: RobotConfig
    # Limit the maximum frames per second.
    fps: int = 100
    teleop_time_s: float | None = None
    # Display all cameras on screen
    display_data: bool = False
    # Display data on a remote Rerun server
    display_ip: str | None = None
    # Port of the remote Rerun server
    display_port: int | None = None
    # Whether to  display compressed images in Rerun
    display_compressed_images: bool = False
    # Global fallback debug flags for teleop feedback printing.
    # These are applied onto teleop config at runtime and are useful when
    # nested `--teleop.*` flags are not propagated in some launcher setups.
    debug_feedback: bool | None = None
    debug_feedback_realtime: bool | None = None
    debug_feedback_realtime_hz: float | None = None


def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    display_data: bool = False,
    duration: float | None = None,
    display_compressed_images: bool = False,
):
    """
    This function continuously reads actions from a teleoperation device, processes them through optional
    pipelines, sends them to a robot, and optionally displays the robot's state. The loop runs at a
    specified frequency until a set duration is reached or it is manually interrupted.

    Args:
        teleop: The teleoperator device instance providing control actions.
        robot: The robot instance being controlled.
        fps: The target frequency for the control loop in frames per second.
        display_data: If True, fetches robot observations and displays them in the console and Rerun.
        display_compressed_images: If True, compresses images before sending them to Rerun for display.
        duration: The maximum duration of the teleoperation loop in seconds. If None, the loop runs indefinitely.
        teleop_action_processor: An optional pipeline to process raw actions from the teleoperator.
        robot_action_processor: An optional pipeline to process actions before they are sent to the robot.
        robot_observation_processor: An optional pipeline to process raw observations from the robot.
    """

    display_len = max(len(key) for key in robot.action_features)
    realtime_feedback_enabled = bool(
        getattr(teleop, "_realtime_debug_enabled", False)
        or getattr(getattr(teleop, "config", object()), "debug_feedback_realtime", False)
    )
    start = time.perf_counter()
    while True:
        loop_start = time.perf_counter()

        # Get robot observation
        # Not really needed for now other than for visualization
        # teleop_action_processor can take None as an observation
        # given that it is the identity processor as default
        obs = robot.get_observation()

        if teleop.feedback_features:
            feedback = {
                key: float(obs[key])
                for key in teleop.feedback_features
                if key in obs and isinstance(obs[key], Real)
            }
            if feedback:
                teleop.send_feedback(feedback)

        # Get teleop action
        raw_action = teleop.get_action()

        # Process teleop action through pipeline
        teleop_action = teleop_action_processor((raw_action, obs))

        # Process action for robot through pipeline
        robot_action_to_send = robot_action_processor((teleop_action, obs))

        # Send processed action to robot (robot_action_processor.to_output should return RobotAction)
        _ = robot.send_action(robot_action_to_send)

        if display_data:
            # Process robot observation through pipeline
            obs_transition = robot_observation_processor(obs)

            log_rerun_data(
                observation=obs_transition,
                action=teleop_action,
                compress_images=display_compressed_images,
            )

            print("\n" + "-" * (display_len + 10))
            print(f"{'NAME':<{display_len}} | {'NORM':>7}")
            # Display the final robot action that was sent
            for motor, value in robot_action_to_send.items():
                print(f"{motor:<{display_len}} | {value:>7.2f}")
            if not realtime_feedback_enabled:
                move_cursor_up(len(robot_action_to_send) + 3)

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(1 / fps - dt_s, 0.0))
        loop_s = time.perf_counter() - loop_start
        if not realtime_feedback_enabled:
            print(f"Teleop loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")
            move_cursor_up(1)

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def teleoperate(cfg: TeleoperateConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    # Compatibility override: some parser paths may miss subclass-only nested
    # flags. Apply raw CLI values directly when provided.
    if isinstance(cfg.teleop, TeleoperatorConfig):
        raw_debug_realtime = parser.parse_arg("teleop.debug_feedback_realtime")
        if raw_debug_realtime is not None and hasattr(cfg.teleop, "debug_feedback_realtime"):
            cfg.teleop.debug_feedback_realtime = raw_debug_realtime.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }

        raw_debug_hz = parser.parse_arg("teleop.debug_feedback_realtime_hz")
        if raw_debug_hz is not None and hasattr(cfg.teleop, "debug_feedback_realtime_hz"):
            cfg.teleop.debug_feedback_realtime_hz = float(raw_debug_hz)

        raw_debug_feedback = parser.parse_arg("teleop.debug_feedback")
        if raw_debug_feedback is not None and hasattr(cfg.teleop, "debug_feedback"):
            cfg.teleop.debug_feedback = raw_debug_feedback.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }

        # Also support top-level flags as a robust fallback.
        # Precedence: explicit top-level arg > explicit teleop.* arg > config default.
        raw_global_debug_feedback = parser.parse_arg("debug_feedback")
        if raw_global_debug_feedback is not None and hasattr(cfg.teleop, "debug_feedback"):
            cfg.teleop.debug_feedback = raw_global_debug_feedback.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }
        elif cfg.debug_feedback is not None and hasattr(cfg.teleop, "debug_feedback"):
            cfg.teleop.debug_feedback = bool(cfg.debug_feedback)

        raw_global_debug_realtime = parser.parse_arg("debug_feedback_realtime")
        if raw_global_debug_realtime is not None and hasattr(cfg.teleop, "debug_feedback_realtime"):
            cfg.teleop.debug_feedback_realtime = raw_global_debug_realtime.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }
        elif cfg.debug_feedback_realtime is not None and hasattr(cfg.teleop, "debug_feedback_realtime"):
            cfg.teleop.debug_feedback_realtime = bool(cfg.debug_feedback_realtime)

        raw_global_debug_hz = parser.parse_arg("debug_feedback_realtime_hz")
        if raw_global_debug_hz is not None and hasattr(cfg.teleop, "debug_feedback_realtime_hz"):
            cfg.teleop.debug_feedback_realtime_hz = float(raw_global_debug_hz)
        elif cfg.debug_feedback_realtime_hz is not None and hasattr(cfg.teleop, "debug_feedback_realtime_hz"):
            cfg.teleop.debug_feedback_realtime_hz = float(cfg.debug_feedback_realtime_hz)

    if cfg.display_data:
        init_rerun(session_name="teleoperation", ip=cfg.display_ip, port=cfg.display_port)
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    if hasattr(teleop, "config"):
        realtime = getattr(teleop.config, "debug_feedback_realtime", None)
        realtime_hz = getattr(teleop.config, "debug_feedback_realtime_hz", None)
        debug_feedback = getattr(teleop.config, "debug_feedback", None)
        if any(v is not None for v in (debug_feedback, realtime, realtime_hz)):
            logging.info(
                "Teleop debug settings | "
                f"debug_feedback={debug_feedback} "
                f"debug_feedback_realtime={realtime} "
                f"debug_feedback_realtime_hz={realtime_hz}"
            )

    teleop.connect()
    robot.connect()

    try:
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            display_compressed_images=display_compressed_images,
        )
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        teleop.disconnect()
        robot.disconnect()


def main():
    register_third_party_plugins()
    teleoperate()


if __name__ == "__main__":
    main()
