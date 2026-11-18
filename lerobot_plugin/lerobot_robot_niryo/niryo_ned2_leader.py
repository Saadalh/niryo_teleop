"""LeRobot teleoperator adapter for the hand-guided master Ned2."""
from __future__ import annotations

from typing import Dict

from lerobot.lerobot_types import RobotAction
from lerobot.teleoperators import Teleoperator

from .config_niryo_ned2 import (
    NiryoNed2LeaderConfig,
    joint_feature_names,
)
from .ros_io import RosBridge, ensure_rclpy


class NiryoNed2Leader(Teleoperator):
    """Reads measured master joint positions and clamps them to Ned2 limits.

    Clamping happens here, before the action reaches the dataset, so the
    recorded ``action`` is exactly what the follower publishes.
    """

    config_class = NiryoNed2LeaderConfig
    name = 'niryo_ned2_leader'

    def __init__(self, config: NiryoNed2LeaderConfig) -> None:
        super().__init__(config)
        self.config = config
        self._bridge = None

    @property
    def action_features(self) -> Dict[str, type]:
        return {name: float for name in joint_feature_names(self.config.joint_names)}

    @property
    def feedback_features(self) -> Dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._bridge is not None

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if self._bridge is not None:
            return
        ensure_rclpy()
        self._bridge = RosBridge(
            'niryo_ned2_leader',
            joint_names=self.config.joint_names,
            master_topic=self.config.master_topic,
            actor_state_topic=self.config.actor_state_topic,
            actor_cmd_topic=self.config.actor_cmd_topic,
            qos_depth=self.config.qos_depth,
            time_from_start_sec=self.config.time_from_start_sec,
        )

    @property
    def is_calibrated(self) -> bool:
        # Calibration is performed by pyniryo before collection starts.
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_action(self) -> RobotAction:
        if self._bridge is None:
            raise RuntimeError('leader is not connected')
        positions = self._bridge.master_positions(
            self.config.stale_timeout_sec
        )
        if positions is None:
            raise RuntimeError(
                f'no fresh master state on {self.config.master_topic}'
            )
        clamped = [
            min(max(value, low), high)
            for value, low, high in zip(
                positions,
                self.config.joint_limits_min,
                self.config.joint_limits_max,
            )
        ]
        return {
            name: float(value)
            for name, value in zip(
                joint_feature_names(self.config.joint_names), clamped
            )
        }

    def send_feedback(self, feedback: dict) -> None:
        del feedback

    def wait_for_master_state(self, timeout_sec: float) -> bool:
        if self._bridge is None:
            raise RuntimeError('leader is not connected')
        return self._bridge.wait_for_master(
            timeout_sec, self.config.stale_timeout_sec
        )

    def disconnect(self) -> None:
        if self._bridge is not None:
            self._bridge.close()
            self._bridge = None
