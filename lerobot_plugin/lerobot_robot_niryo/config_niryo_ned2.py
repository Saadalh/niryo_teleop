"""Configuration and registration for the Niryo Ned2 LeRobot adapters."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig

NED2_JOINT_NAMES = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6',
]

# Ned2 command limits (radians), mirroring teleop_params.yaml.
NED2_LIMITS_MIN = [
    -2.999871918328051,
    -1.8325957145941667,
    -1.3400637996813345,
    -2.090031779263347,
    -1.9200367101190883,
    -2.530029283691147,
]
NED2_LIMITS_MAX = [
    2.999871918328051,
    0.6101671064972578,
    1.4922565104552499,
    2.090031779263347,
    1.9228292369222795,
    2.530029283691147,
]


def joint_feature_names(joint_names: List[str]) -> List[str]:
    """LeRobot hardware feature keys for the joint positions."""
    return [f'{name}.pos' for name in joint_names]


@dataclass
class NiryoNed2CommonConfig:
    """Settings shared by the leader (teleoperator) and follower (robot)."""

    master_topic: str = '/master/joint_states'
    actor_state_topic: str = '/actor/joint_states'
    actor_cmd_topic: str = '/actor/joint_cmd'
    joint_names: List[str] = field(
        default_factory=lambda: list(NED2_JOINT_NAMES)
    )
    joint_limits_min: List[float] = field(
        default_factory=lambda: list(NED2_LIMITS_MIN)
    )
    joint_limits_max: List[float] = field(
        default_factory=lambda: list(NED2_LIMITS_MAX)
    )
    qos_depth: int = 10
    stale_timeout_sec: float = 0.5
    # Outgoing single-point horizon. The actor controller floors and zeroes it
    # before the robot action server, which is what the hardware validated.
    time_from_start_sec: float = 0.2


@RobotConfig.register_subclass('niryo_ned2_follower')
@dataclass
class NiryoNed2FollowerConfig(RobotConfig, NiryoNed2CommonConfig):
    """LeRobot robot adapter for the actor Ned2."""

    id: str = 'niryo_actor'
    cameras: Dict[str, CameraConfig] = field(default_factory=dict)


@TeleoperatorConfig.register_subclass('niryo_ned2_leader')
@dataclass
class NiryoNed2LeaderConfig(TeleoperatorConfig, NiryoNed2CommonConfig):
    """LeRobot teleoperator adapter for the hand-guided master Ned2."""

    id: str = 'niryo_master'
