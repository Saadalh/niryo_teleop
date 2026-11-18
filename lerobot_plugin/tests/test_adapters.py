"""Leader/follower adapter behaviour with a fake ROS bridge (no ROS graph)."""
import pytest

pytest.importorskip('lerobot')
pytest.importorskip('rclpy')

from lerobot_robot_niryo.config_niryo_ned2 import (  # noqa: E402
    NiryoNed2FollowerConfig,
    NiryoNed2LeaderConfig,
)
from lerobot_robot_niryo.niryo_ned2_follower import NiryoNed2Follower  # noqa: E402
from lerobot_robot_niryo.niryo_ned2_leader import NiryoNed2Leader  # noqa: E402

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


class _FakeBridge:
    def __init__(self):
        self.published = []
        self.master = None
        self.actor = None
        self.status = 'LIVE_TELEOP'

    def master_positions(self, stale_timeout_sec=0.5):
        del stale_timeout_sec
        return None if self.master is None else list(self.master)

    def actor_positions(self, stale_timeout_sec=0.5):
        del stale_timeout_sec
        return None if self.actor is None else list(self.actor)

    def publish_action(self, positions):
        self.published.append(list(positions))

    def latest_status(self):
        return self.status

    def close(self):
        self.closed = True


def _leader():
    return NiryoNed2Leader(NiryoNed2LeaderConfig())


def test_leader_clamps_master_positions_into_limits():
    leader = _leader()
    bridge = _FakeBridge()
    leader._bridge = bridge
    bridge.master = [99.0, -99.0, 0.0, 0.0, 0.0, 0.0]

    action = leader.get_action()

    assert action['joint_1.pos'] == pytest.approx(
        leader.config.joint_limits_max[0]
    )
    assert action['joint_2.pos'] == pytest.approx(
        leader.config.joint_limits_min[1]
    )
    assert action['joint_3.pos'] == pytest.approx(0.0)


def test_leader_raises_without_a_fresh_master_sample():
    leader = _leader()
    leader._bridge = _FakeBridge()
    with pytest.raises(RuntimeError, match='master'):
        leader.get_action()


def test_follower_sends_action_unchanged_to_the_actor_command_topic():
    follower = NiryoNed2Follower(NiryoNed2FollowerConfig())
    bridge = _FakeBridge()
    follower._bridge = bridge
    action = {f'joint_{i}.pos': 0.1 * i for i in range(1, 7)}

    returned = follower.send_action(action)

    assert bridge.published == [
        [0.1 * i for i in range(1, 7)]
    ]
    assert returned == action


def test_follower_rejects_actions_missing_a_joint():
    follower = NiryoNed2Follower(NiryoNed2FollowerConfig())
    follower._bridge = _FakeBridge()
    with pytest.raises(ValueError, match='missing action field'):
        follower.send_action({'joint_1.pos': 0.0})


def test_follower_observation_reads_measured_actor_state():
    follower = NiryoNed2Follower(NiryoNed2FollowerConfig())
    bridge = _FakeBridge()
    follower._bridge = bridge
    bridge.actor = [0.2] * 6

    observation = follower.get_observation()

    assert observation == {f'joint_{i}.pos': 0.2 for i in range(1, 7)}


def test_follower_raises_without_a_fresh_actor_sample():
    follower = NiryoNed2Follower(NiryoNed2FollowerConfig())
    follower._bridge = _FakeBridge()
    with pytest.raises(RuntimeError, match='actor'):
        follower.get_observation()
