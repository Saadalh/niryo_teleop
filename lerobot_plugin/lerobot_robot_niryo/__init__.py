"""LeRobot adapters for the Niryo Ned2 teleoperation stack.

Importing this package registers the two adapters with LeRobot:

- ``niryo_ned2_follower`` (``Robot``): actor measured state + ``/actor/joint_cmd``
- ``niryo_ned2_leader`` (``Teleoperator``): master measured state, clamped to
  the Ned2 joint limits

ROS runtime nodes stay separate processes; these adapters only use the public
topics. See ``docs/DATA_COLLECTION.md`` in the repository root.
"""
from .config_niryo_ned2 import (  # noqa: F401
    NiryoNed2FollowerConfig,
    NiryoNed2LeaderConfig,
)
from .niryo_ned2_follower import NiryoNed2Follower  # noqa: F401
from .niryo_ned2_leader import NiryoNed2Leader  # noqa: F401

__all__ = [
    'NiryoNed2Follower',
    'NiryoNed2FollowerConfig',
    'NiryoNed2Leader',
    'NiryoNed2LeaderConfig',
]
