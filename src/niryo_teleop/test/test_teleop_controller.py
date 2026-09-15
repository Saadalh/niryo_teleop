"""Unit tests for the Chunk 4 teleoperation controller (no robot, no spin)."""
import math

import pytest
import yaml
from sensor_msgs.msg import JointState

from niryo_teleop.teleop.teleop_controller_node import (
    NED2_JOINT_NAMES,
    NED2_LIMITS_MAX,
    NED2_LIMITS_MIN,
    TeleoperationController,
    TeleopControllerNode,
    apply_joint_limits,
    build_actor_command,
    map_master_to_actor,
    validate_master_state,
)


def _make_state(positions, names=None, stamp_sec=100.0):
    msg = JointState()
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
    msg.name = list(names) if names is not None else list(NED2_JOINT_NAMES)
    msg.position = list(positions)
    return msg


def test_node_alias():
    assert TeleoperationController is TeleopControllerNode


def test_identity_mapping():
    pos = [0.1, -0.2, 0.3, 0.0, 0.5, -0.5]
    assert map_master_to_actor(pos, [1.0] * 6, [0.0] * 6) == pytest.approx(pos)


def test_offset_mapping():
    pos = [0.1, -0.2, 0.3, 0.0, 0.5, -0.5]
    out = map_master_to_actor(pos, [1.0] * 6, [0.1] * 6)
    assert out == pytest.approx([p + 0.1 for p in pos])


def test_scale_mapping_and_inversion():
    pos = [0.5, -0.5, 1.0, 0.0, 0.25, -0.25]
    assert map_master_to_actor(pos, [2.0] * 6, [0.0] * 6) == pytest.approx(
        [2 * p for p in pos]
    )
    # direction inversion via negative scale
    assert map_master_to_actor(pos, [-1.0] * 6, [0.0] * 6) == pytest.approx(
        [-p for p in pos]
    )


def test_mapping_size_mismatch_raises():
    with pytest.raises(ValueError):
        map_master_to_actor([0.0] * 6, [1.0] * 5, [0.0] * 6)


def test_order_validation_strict_vs_relaxed():
    pos = [0.0] * 6
    shuffled = list(reversed(NED2_JOINT_NAMES))
    msg = _make_state(pos, names=shuffled)
    ok, _ = validate_master_state(msg, NED2_JOINT_NAMES, 0.5, 100.0, 100.0, True)
    assert not ok
    ok_relaxed, _ = validate_master_state(
        msg, NED2_JOINT_NAMES, 0.5, 100.0, 100.0, False
    )
    assert ok_relaxed


def test_invalid_joint_count_rejected():
    msg = _make_state([0.0] * 5, names=NED2_JOINT_NAMES[:5])
    ok, reason = validate_master_state(msg, NED2_JOINT_NAMES, 0.5, 100.0, 100.0)
    assert not ok and 'count' in reason
    # name/position size mismatch is also rejected
    bad = _make_state([0.0] * 5)
    ok_bad, _ = validate_master_state(bad, NED2_JOINT_NAMES, 0.5, 100.0, 100.0)
    assert not ok_bad


def test_nonfinite_rejected():
    msg = _make_state([0.0, math.inf, 0.0, 0.0, 0.0, 0.0])
    ok, _ = validate_master_state(msg, NED2_JOINT_NAMES, 0.5, 100.0, 100.0)
    assert not ok


def test_joint_limits_clamp_and_passthrough():
    inside = [0.0] * 6
    clamped, did = apply_joint_limits(inside, NED2_LIMITS_MIN, NED2_LIMITS_MAX)
    assert not did and clamped == inside
    over = [10.0, -10.0, 0.0, 0.0, 0.0, 0.0]
    clamped, did = apply_joint_limits(over, NED2_LIMITS_MIN, NED2_LIMITS_MAX)
    assert did
    assert clamped[0] == pytest.approx(NED2_LIMITS_MAX[0])
    assert clamped[1] == pytest.approx(NED2_LIMITS_MIN[1])
    assert clamped[2:] == [0.0] * 4


def test_stale_stamp_rejected_zero_stamp_uses_rx_time():
    fresh = _make_state([0.0] * 6, stamp_sec=100.0)
    ok, _ = validate_master_state(fresh, NED2_JOINT_NAMES, 0.5, 100.2, 100.2)
    assert ok
    old = _make_state([0.0] * 6, stamp_sec=90.0)
    ok_old, reason = validate_master_state(old, NED2_JOINT_NAMES, 0.5, 100.0, 100.0)
    assert not ok_old and 'stale' in reason
    # unset stamp (0/0) falls back to receive time instead of false-stale
    nostamp = _make_state([0.0] * 6, stamp_sec=0.0)
    ok_ns, _ = validate_master_state(nostamp, NED2_JOINT_NAMES, 0.5, 100.1, 100.0)
    assert ok_ns


def test_build_actor_command_fields():
    from builtin_interfaces.msg import Time
    stamp = Time(sec=7, nanosec=8)
    cmd = build_actor_command(NED2_JOINT_NAMES, [0.1] * 6, stamp, 0.2)
    assert list(cmd.joint_names) == NED2_JOINT_NAMES
    assert len(cmd.points) == 1
    assert list(cmd.points[0].positions) == [0.1] * 6
    assert cmd.points[0].time_from_start.sec == 0
    assert cmd.points[0].time_from_start.nanosec == pytest.approx(2e8)
    assert cmd.header.stamp.sec == 7 and cmd.header.stamp.nanosec == 8


def test_teleop_params_yaml_identity_defaults():
    import pathlib
    cfg = pathlib.Path(__file__).resolve().parent.parent / 'config' / 'teleop_params.yaml'
    data = yaml.safe_load(cfg.read_text())
    params = data['teleoperation_controller']['ros__parameters']
    assert params['master_topic'] == '/master/joint_states'
    assert params['actor_topic'] == '/actor/joint_cmd'
    assert params['joint_names'] == NED2_JOINT_NAMES
    assert params['joint_scale'] == [1.0] * 6
    assert params['joint_offsets'] == [0.0] * 6
    assert params['joint_limits_min'] == pytest.approx(NED2_LIMITS_MIN)
    assert params['joint_limits_max'] == pytest.approx(NED2_LIMITS_MAX)
    assert float(params['control_rate_hz']) > 0
    assert float(params['maximum_state_age_sec']) > 0
