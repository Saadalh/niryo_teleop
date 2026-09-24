"""Unit tests for the actor joint-state relay (no robot, no spin)."""
import math

from sensor_msgs.msg import JointState

from niryo_teleop.actor.actor_joint_state_publisher import ActorJointStatePublisher
from niryo_teleop.master.master_joint_state_publisher import (
    NED2_JOINT_NAMES,
    validate_joint_state,
)


def _make_msg(positions, names=None):
    msg = JointState()
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 34
    msg.header.frame_id = 'base_link'
    msg.name = list(names) if names is not None else list(NED2_JOINT_NAMES)
    msg.position = list(positions)
    return msg


def _make_node():
    import rclpy

    rclpy.init()
    return ActorControllerShim()


class ActorControllerShim(ActorJointStatePublisher):
    """Test helper: ActorJointStatePublisher with recordable publishes."""

    def __init__(self):
        super().__init__()
        self.published = []
        self._pub.publish = self.published.append


def _teardown_node(node):
    import rclpy

    node.destroy_node()
    rclpy.shutdown()


def test_relay_preserves_positions_order():
    node = _make_node()
    try:
        node._on_joint_state(_make_msg([0.1, -0.2, 0.3, 0.0, 0.5, -0.5]))
        assert len(node.published) == 1
        out = node.published[0]
        assert list(out.name) == NED2_JOINT_NAMES
        assert list(out.position) == [0.1, -0.2, 0.3, 0.0, 0.5, -0.5]
    finally:
        _teardown_node(node)


def test_relay_restamps_locally():
    node = _make_node()
    try:
        node._on_joint_state(_make_msg([0.0] * 6))
        out = node.published[0]
        assert (out.header.stamp.sec, out.header.stamp.nanosec) != (12, 34)
        assert out.header.frame_id == 'base_link'
    finally:
        _teardown_node(node)


def test_relay_is_one_to_one_without_limiting():
    node = _make_node()
    try:
        node._on_joint_state(_make_msg([0.1] * 6))
        node._on_joint_state(_make_msg([0.2] * 6))
        assert len(node.published) == 2
        assert list(node.published[1].position) == [0.2] * 6
    finally:
        _teardown_node(node)


def test_relay_drops_invalid():
    node = _make_node()
    try:
        node._on_joint_state(_make_msg([0.0, math.nan, 0.0, 0.0, 0.0, 0.0]))
        assert node.published == []
        ok, _ = validate_joint_state(
            _make_msg([0.0] * 5), NED2_JOINT_NAMES, True
        )
        assert not ok
    finally:
        _teardown_node(node)
