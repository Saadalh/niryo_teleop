"""LeRobot robot adapter for the actor Ned2."""
from __future__ import annotations

from typing import Dict

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.robots import Robot

from .config_niryo_ned2 import (
    NiryoNed2FollowerConfig,
    joint_feature_names,
)
from .ros_io import RosBridge, ensure_rclpy


class NiryoNed2Follower(Robot):
    """Reads actor joint state and streams commands to ``/actor/joint_cmd``.

    The ROS relays and actor controller stay separate processes; this adapter
    only uses the public topics, which keeps the LeRobot plugin independent of
    ROS node internals.
    """

    config_class = NiryoNed2FollowerConfig
    name = 'niryo_ned2_follower'

    def __init__(self, config: NiryoNed2FollowerConfig) -> None:
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._bridge = None

    @property
    def _joint_features(self) -> Dict[str, type]:
        return {name: float for name in joint_feature_names(self.config.joint_names)}

    @property
    def _camera_features(self) -> Dict[str, tuple]:
        features: Dict[str, tuple] = {}
        for name, camera in self.cameras.items():
            features[name] = (camera.height, camera.width, 3)
        return features

    @property
    def observation_features(self) -> Dict[str, type | tuple]:
        return {**self._joint_features, **self._camera_features}

    @property
    def action_features(self) -> Dict[str, type]:
        return self._joint_features

    @property
    def is_connected(self) -> bool:
        return (
            self._bridge is not None
            and all(camera.is_connected for camera in self.cameras.values())
        )

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if self._bridge is not None:
            return
        ensure_rclpy()
        self._bridge = RosBridge(
            'niryo_ned2_follower',
            joint_names=self.config.joint_names,
            master_topic=self.config.master_topic,
            actor_state_topic=self.config.actor_state_topic,
            actor_cmd_topic=self.config.actor_cmd_topic,
            qos_depth=self.config.qos_depth,
            time_from_start_sec=self.config.time_from_start_sec,
        )
        for camera in self.cameras.values():
            if not camera.is_connected:
                camera.connect()

    @property
    def is_calibrated(self) -> bool:
        # Calibration is performed by pyniryo before collection starts.
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_observation(self) -> RobotObservation:
        if self._bridge is None:
            raise RuntimeError('follower is not connected')
        positions = self._bridge.actor_positions(
            self.config.stale_timeout_sec
        )
        if positions is None:
            raise RuntimeError(
                f'no fresh actor state on {self.config.actor_state_topic}'
            )
        observation: RobotObservation = {
            name: float(value)
            for name, value in zip(
                joint_feature_names(self.config.joint_names), positions
            )
        }
        for name, camera in self.cameras.items():
            observation[name] = camera.async_read(timeout_ms=500)
        return observation

    def send_action(self, action: RobotAction) -> RobotAction:
        if self._bridge is None:
            raise RuntimeError('follower is not connected')
        names = joint_feature_names(self.config.joint_names)
        try:
            positions = [float(action[name]) for name in names]
        except KeyError as exc:
            raise ValueError(f'missing action field {exc}') from exc
        self._bridge.publish_action(positions)
        return {name: value for name, value in zip(names, positions)}

    def enable_controller(self, timeout_sec: float = 5.0) -> tuple[bool, str]:
        """Enable the gated actor controller after bench preparation."""
        if self._bridge is None:
            raise RuntimeError('follower is not connected')
        return self._bridge.enable_controller(timeout_sec)

    def latest_status(self):
        if self._bridge is None:
            return None
        return self._bridge.latest_status()

    def wait_for_actor_state(self, timeout_sec: float) -> bool:
        if self._bridge is None:
            raise RuntimeError('follower is not connected')
        return self._bridge.wait_for_actor(
            timeout_sec, self.config.stale_timeout_sec
        )

    def disconnect(self) -> None:
        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()
        if self._bridge is not None:
            self._bridge.close()
            self._bridge = None
