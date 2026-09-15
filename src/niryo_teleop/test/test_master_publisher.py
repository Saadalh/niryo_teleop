"""Unit tests for master_joint_state_publisher logic (no robot, no spin)."""
import math

import yaml
from sensor_msgs.msg import JointState

from niryo_teleop.master.master_joint_state_publisher import (
    NED2_JOINT_NAMES,
    build_master_msg,
    validate_joint_state,
)


def _make_msg(positions, names=None, velocity=None, effort=None):
    msg = JointState()
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 34
    msg.header.frame_id = 'base_link'
    msg.name = list(names) if names is not None else list(NED2_JOINT_NAMES)
    msg.position = list(positions)
    msg.velocity = list(velocity) if velocity is not None else []
    msg.effort = list(effort) if effort is not None else []
    return msg


def test_ned2_interface_constants():
    assert NED2_JOINT_NAMES == [
        'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6',
    ]


def test_valid_full_message_passes_and_preserves_stamp_order_units():
    pos = [0.1, -0.2, 0.3, 0.0, 0.5, -0.5]
    vel = [0.01] * 6
    eff = [1.0] * 6
    msg = _make_msg(pos, velocity=vel, effort=eff)
    ok, _ = validate_joint_state(msg, NED2_JOINT_NAMES)
    assert ok
    out = build_master_msg(msg, NED2_JOINT_NAMES)
    assert list(out.name) == NED2_JOINT_NAMES  # ordering preserved
    assert list(out.position) == pos  # radians, untouched
    assert list(out.velocity) == vel
    assert list(out.effort) == eff
    assert out.header.stamp.sec == 12 and out.header.stamp.nanosec == 34


def test_missing_velocity_effort_not_fabricated():
    msg = _make_msg([0.0] * 6)
    ok, _ = validate_joint_state(msg, NED2_JOINT_NAMES)
    assert ok
    out = build_master_msg(msg, NED2_JOINT_NAMES)
    assert list(out.velocity) == []
    assert list(out.effort) == []


def test_mismatched_velocity_effort_cleared_position_kept():
    msg = _make_msg([0.0] * 6, velocity=[0.1, 0.2], effort=[1.0])
    ok, _ = validate_joint_state(msg, NED2_JOINT_NAMES)
    assert ok  # validation is about positions, not aux fields
    out = build_master_msg(msg, NED2_JOINT_NAMES)
    assert list(out.position) == [0.0] * 6
    assert list(out.velocity) == []
    assert list(out.effort) == []


def test_nan_position_rejected():
    msg = _make_msg([0.0, math.nan, 0.0, 0.0, 0.0, 0.0])
    ok, reason = validate_joint_state(msg, NED2_JOINT_NAMES)
    assert not ok and 'finite' in reason


def test_size_mismatch_rejected():
    msg = _make_msg([0.0] * 5)
    ok, _ = validate_joint_state(msg, NED2_JOINT_NAMES)
    assert not ok
    msg2 = _make_msg([0.0] * 6, names=['joint_1', 'joint_2'])
    ok2, _ = validate_joint_state(msg2, NED2_JOINT_NAMES)
    assert not ok2


def test_wrong_names_strict_vs_relaxed():
    other = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    msg = _make_msg([0.0] * 6, names=other)
    ok_strict, _ = validate_joint_state(msg, NED2_JOINT_NAMES, True)
    assert not ok_strict
    ok_relaxed, _ = validate_joint_state(msg, NED2_JOINT_NAMES, False)
    assert ok_relaxed
    out = build_master_msg(msg, NED2_JOINT_NAMES)
    assert list(out.name) == other  # never silently remapped


def test_master_params_yaml_valid():
    import pathlib
    cfg = pathlib.Path(__file__).resolve().parent.parent / 'config' / 'master_params.yaml'
    data = yaml.safe_load(cfg.read_text())
    params = data['master_joint_state_publisher']['ros__parameters']
    assert params['output_topic'] == '/master/joint_states'
    assert params['joint_names'] == NED2_JOINT_NAMES
    assert float(params['publish_rate_hz']) > 0
    assert float(params['stale_timeout_sec']) > 0
