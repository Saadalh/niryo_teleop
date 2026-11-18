"""End-to-end dataset schema check through LeRobot's record_loop (no ROS)."""
import numpy as np
import pytest

pytest.importorskip('lerobot')
pytest.importorskip('rclpy')

from lerobot.datasets import LeRobotDataset, VideoEncodingManager  # noqa: E402
from lerobot.processor import make_default_processors  # noqa: E402
from lerobot.scripts.lerobot_record import record_loop  # noqa: E402
from lerobot_robot_niryo.collect import build_dataset_features  # noqa: E402
from lerobot_robot_niryo.config_niryo_ned2 import (  # noqa: E402
    NiryoNed2LeaderConfig,
)
from lerobot_robot_niryo.niryo_ned2_leader import NiryoNed2Leader  # noqa: E402

JOINT_NAMES = [f'joint_{i}.pos' for i in range(1, 7)]
IMAGE_SHAPE = (24, 32, 3)


class _FakeBridge:
    def __init__(self, master):
        self.master = master
        self.published = []

    def master_positions(self, stale_timeout_sec=0.5):
        del stale_timeout_sec
        return list(self.master)

    def publish_action(self, positions):
        self.published.append(list(positions))


class _FakeRobot:
    """Minimal robot exposing the LeRobot Robot surface record_loop uses."""

    name = 'niryo_ned2_follower'

    def __init__(self):
        self.sent = []
        self.observation = {name: 0.2 for name in JOINT_NAMES}
        self.observation['front'] = np.zeros(IMAGE_SHAPE, dtype=np.uint8)
        self.cameras = {'front': object()}

    @property
    def action_features(self):
        return {name: float for name in JOINT_NAMES}

    @property
    def observation_features(self):
        return {**self.action_features, 'front': IMAGE_SHAPE}

    def get_observation(self):
        return dict(self.observation)

    def send_action(self, action):
        self.sent.append([action[name] for name in JOINT_NAMES])
        return dict(action)


def _leader_with_master(master):
    leader = NiryoNed2Leader(NiryoNed2LeaderConfig())
    bridge = _FakeBridge(master)
    leader._bridge = bridge
    return leader, bridge


def _clamp(leader, positions):
    return [
        min(max(value, low), high)
        for value, low, high in zip(
            positions,
            leader.config.joint_limits_min,
            leader.config.joint_limits_max,
        )
    ]


def test_recorded_episode_matches_actor_state_and_issued_command(tmp_path):
    robot = _FakeRobot()
    # Out-of-limit master: the leader must clamp before the action is recorded.
    master = [99.0, -99.0, 0.3, -0.4, 0.5, -0.6]
    leader, _bridge = _leader_with_master(master)
    processors = make_default_processors()

    features = build_dataset_features(robot, leader, processors)
    assert features['action']['dtype'] == 'float32'
    assert tuple(features['action']['shape']) == (6,)
    assert features['observation.state']['dtype'] == 'float32'
    assert tuple(features['observation.state']['shape']) == (6,)
    assert features['observation.images.front']['dtype'] == 'video'
    assert tuple(features['observation.images.front']['shape']) == IMAGE_SHAPE

    dataset = LeRobotDataset.create(
        repo_id='local/roundtrip',
        fps=10,
        root=tmp_path / 'roundtrip',
        robot_type=robot.name,
        features=features,
        use_videos=True,
        image_writer_threads=2,
    )
    with VideoEncodingManager(dataset):
        record_loop(
            robot=robot,
            events={'exit_early': False},
            fps=10,
            teleop_action_processor=processors[0],
            robot_action_processor=processors[1],
            robot_observation_processor=processors[2],
            teleop=leader,
            dataset=dataset,
            control_time_s=0.35,
            single_task='test episode',
        )
        dataset.save_episode()

    assert dataset.num_episodes == 1
    frames = dataset.num_frames
    assert 2 <= frames <= 6

    expected_action = _clamp(leader, master)
    assert len(robot.sent) >= 1

    reopened = LeRobotDataset('local/roundtrip', root=tmp_path / 'roundtrip')
    assert reopened.num_episodes == 1
    assert reopened.num_frames == frames
    assert reopened.fps == 10

    sample = reopened[0]
    assert sample['observation.state'].shape == (6,)
    np.testing.assert_allclose(
        sample['observation.state'].numpy(), [0.2] * 6, rtol=0, atol=1e-6
    )
    np.testing.assert_allclose(
        sample['action'].numpy(), expected_action, rtol=0, atol=1e-6
    )
    assert tuple(sample['observation.images.front'].shape) == (
        3,
        *IMAGE_SHAPE[:2],
    )
    assert sample['task'] == 'test episode'

    # The action stored in the dataset is the command the robot received.
    for sent in robot.sent:
        assert sent == pytest.approx(expected_action)
