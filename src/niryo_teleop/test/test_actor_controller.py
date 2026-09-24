"""Unit tests for the actor controller (no robot, no action server)."""
import math

import pytest
import yaml
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from niryo_teleop.actor.actor_controller import (
    ActorController,
    apply_min_duration,
    build_init_trajectory,
    max_abs_diff,
    target_positions,
    validate_actor_command,
)
from niryo_teleop.master.master_joint_state_publisher import NED2_JOINT_NAMES


def _make_cmd(positions_list, names=None):
    msg = JointTrajectory()
    msg.joint_names = list(names) if names is not None else list(NED2_JOINT_NAMES)
    for positions in positions_list:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        msg.points.append(point)
    return msg


def _make_measured(positions, names=None):
    msg = JointState()
    msg.header.stamp.sec = 50
    msg.name = list(names) if names is not None else list(NED2_JOINT_NAMES)
    msg.position = list(positions)
    return msg


class _FakeFuture:
    def __init__(self, status):
        self._status = status

    def result(self):
        class _Result:
            pass

        res = _Result()
        res.status = self._status
        return res


def _make_node(monkeypatch):
    """Construct an ActorController with no real action server or sends."""
    import rclpy

    monkeypatch.setattr(ActorController, '_wait_for_server', lambda self, t: True)
    rclpy.init()
    node = ActorController()
    sent = []

    def fake_send(traj, is_init=False):
        # Mirror the real _send_goal gate bookkeeping without touching ROS.
        node._goal_outstanding = True
        node._outstanding_is_init = is_init
        node._goal_sent_sec = node._now_sec()
        node._watchdog_fired = False
        sent.append((traj, is_init))

    monkeypatch.setattr(node, '_send_goal', fake_send)
    return node, sent


def _teardown_node(node):
    import rclpy

    node.destroy_node()
    rclpy.shutdown()


def _give_fresh_cmd_and_measured(node, cmd_pos, meas_pos):
    node._on_joint_cmd(_make_cmd([cmd_pos]))
    node._on_measured_state(_make_measured(meas_pos))


# --- pure validation --------------------------------------------------------


def test_valid_command_accepted():
    ok, _ = validate_actor_command(_make_cmd([[0.1] * 6]), NED2_JOINT_NAMES)
    assert ok


def test_invalid_names_rejected():
    ok, reason = validate_actor_command(
        _make_cmd([[0.0] * 6], names=['j1', 'j2', 'j3', 'j4', 'j5', 'j6']),
        NED2_JOINT_NAMES,
    )
    assert not ok and 'names' in reason


def test_invalid_count_rejected():
    ok, _ = validate_actor_command(_make_cmd([[0.0] * 5]), NED2_JOINT_NAMES)
    assert not ok


def test_nonfinite_rejected():
    ok, _ = validate_actor_command(
        _make_cmd([[0.0, math.inf, 0.0, 0.0, 0.0, 0.0]]), NED2_JOINT_NAMES
    )
    assert not ok


def test_empty_points_rejected():
    ok, _ = validate_actor_command(_make_cmd([]), NED2_JOINT_NAMES)
    assert not ok


def test_init_trajectory_single_point_at_duration():
    from builtin_interfaces.msg import Time

    traj = build_init_trajectory(NED2_JOINT_NAMES, [0.5] * 6, 3.0, Time(sec=1))
    assert len(traj.points) == 1
    assert list(traj.points[0].positions) == [0.5] * 6
    assert traj.points[0].time_from_start.sec == 3
    # no point at t=0: the controller rejects those


def test_apply_min_duration_floors_short_goals():
    cmd = _make_cmd([[0.1] * 6])
    cmd.points[0].time_from_start.sec = 0
    cmd.points[0].time_from_start.nanosec = 200000000
    out = apply_min_duration(cmd, 1.0)
    dur = out.points[0].time_from_start
    assert dur.sec + dur.nanosec * 1e-9 == pytest.approx(1.0)
    assert list(out.points[0].positions) == [0.1] * 6
    assert list(out.joint_names) == NED2_JOINT_NAMES


def test_apply_min_duration_leaves_long_goals():
    cmd = _make_cmd([[0.1] * 6])
    cmd.points[0].time_from_start.sec = 3
    cmd.points[0].time_from_start.nanosec = 0
    out = apply_min_duration(cmd, 1.0)
    assert out.points[0].time_from_start.sec == 3


def test_max_abs_diff():
    assert max_abs_diff([0.0] * 6, [0.1] * 6) == pytest.approx(0.1)
    assert max_abs_diff([1.0] * 6, [1.0] * 6) == pytest.approx(0.0)


# --- latest-only streaming --------------------------------------------------


def test_latest_overwrites_not_queued(monkeypatch):
    node, sent = _make_node(monkeypatch)
    try:
        node._state = 'LIVE_TELEOP'
        node._on_joint_cmd(_make_cmd([[0.1] * 6]))
        node._on_joint_cmd(_make_cmd([[0.2] * 6]))
        node._tick()
        assert len(sent) == 1
        assert list(target_positions(sent[0][0])) == [0.2] * 6
    finally:
        _teardown_node(node)


def test_no_send_without_update(monkeypatch):
    node, sent = _make_node(monkeypatch)
    try:
        node._state = 'LIVE_TELEOP'
        node._tick()
        assert sent == []
    finally:
        _teardown_node(node)


def test_stale_not_sent(monkeypatch):
    node, sent = _make_node(monkeypatch)
    try:
        node._state = 'LIVE_TELEOP'
        node._on_joint_cmd(_make_cmd([[0.1] * 6]))
        node._latest_cmd_rx_sec = node._now_sec() - 10.0
        node._tick()
        assert sent == []
    finally:
        _teardown_node(node)


def test_identical_not_resent(monkeypatch):
    from action_msgs.msg import GoalStatus

    node, sent = _make_node(monkeypatch)
    try:
        node._state = 'LIVE_TELEOP'
        node._on_joint_cmd(_make_cmd([[0.1] * 6]))
        node._tick()
        assert len(sent) == 1
        # first goal completes; identical re-command must be skipped by epsilon
        node._on_goal_result(_FakeFuture(GoalStatus.STATUS_SUCCEEDED), False)
        node._on_joint_cmd(_make_cmd([[0.1] * 6]))
        node._tick()
        assert len(sent) == 1
    finally:
        _teardown_node(node)


def test_live_waits_for_outstanding_goal(monkeypatch):
    from action_msgs.msg import GoalStatus

    node, sent = _make_node(monkeypatch)
    try:
        node._state = 'LIVE_TELEOP'
        node._on_joint_cmd(_make_cmd([[0.1] * 6]))
        node._tick()
        assert len(sent) == 1
        # newer command arrives while the goal executes: kept, not sent
        node._on_joint_cmd(_make_cmd([[0.3] * 6]))
        node._tick()
        assert len(sent) == 1
        # goal completes -> next tick sends the newest target, not the old one
        node._on_goal_result(_FakeFuture(GoalStatus.STATUS_SUCCEEDED), False)
        node._tick()
        assert len(sent) == 2
        assert list(target_positions(sent[1][0])) == [0.3] * 6
    finally:
        _teardown_node(node)


def test_watchdog_rearms_stuck_goal(monkeypatch):
    node, sent = _make_node(monkeypatch)
    try:
        node._state = 'LIVE_TELEOP'
        node._on_joint_cmd(_make_cmd([[0.1] * 6]))
        node._tick()
        assert len(sent) == 1
        # result never arrives: age the goal past the watchdog timeout
        node._goal_sent_sec = node._now_sec() - (node._goal_timeout + 1.0)
        node._tick()
        assert not node._goal_outstanding
        # a fresh command now flows again instead of freezing forever
        node._on_joint_cmd(_make_cmd([[0.4] * 6]))
        node._tick()
        assert len(sent) == 2
        assert list(target_positions(sent[1][0])) == [0.4] * 6
    finally:
        _teardown_node(node)


# --- initialization gating --------------------------------------------------


def test_init_waits_without_command(monkeypatch):
    node, sent = _make_node(monkeypatch)
    try:
        assert node._state == 'INITIALIZING'
        node._on_measured_state(_make_measured([0.0] * 6))
        node._tick()
        assert node._state == 'INITIALIZING'
        assert sent == []
    finally:
        _teardown_node(node)


def test_init_near_enters_live_without_motion(monkeypatch):
    node, sent = _make_node(monkeypatch)
    try:
        _give_fresh_cmd_and_measured(node, [0.1] * 6, [0.11] * 6)
        node._tick()
        assert node._state == 'LIVE_TELEOP'
        assert sent == []
    finally:
        _teardown_node(node)


def test_init_far_sends_init_trajectory(monkeypatch):
    node, sent = _make_node(monkeypatch)
    try:
        _give_fresh_cmd_and_measured(node, [1.0] * 6, [0.0] * 6)
        node._tick()
        assert node._state == 'INITIALIZING'
        assert len(sent) == 1
        traj, is_init = sent[0]
        assert is_init
        assert len(traj.points) == 1
        assert list(traj.points[0].positions) == [1.0] * 6
        assert traj.points[0].time_from_start.sec == 3
    finally:
        _teardown_node(node)


def test_init_failure_stays_gated(monkeypatch):
    from action_msgs.msg import GoalStatus

    node, sent = _make_node(monkeypatch)
    try:
        _give_fresh_cmd_and_measured(node, [1.0] * 6, [0.0] * 6)
        node._tick()
        assert node._goal_outstanding
        node._last_init_attempt_sec = node._now_sec()  # block immediate retry
        node._on_goal_result(_FakeFuture(GoalStatus.STATUS_ABORTED), True)
        assert node._state == 'INITIALIZING'
        assert not node._goal_outstanding
    finally:
        _teardown_node(node)


def test_master_moved_during_init_retargets(monkeypatch):
    from action_msgs.msg import GoalStatus

    node, sent = _make_node(monkeypatch)
    try:
        _give_fresh_cmd_and_measured(node, [1.0] * 6, [0.0] * 6)
        node._tick()  # first init toward 1.0
        assert len(sent) == 1
        node._on_goal_result(_FakeFuture(GoalStatus.STATUS_SUCCEEDED), True)
        # master moved meanwhile: newest target is 1.5, measured still 0.0
        node._on_joint_cmd(_make_cmd([[1.5] * 6]))
        node._last_init_attempt_sec = float('-inf')  # allow retry
        node._tick()
        assert node._state == 'INITIALIZING'  # not live: still far
        assert len(sent) == 2
        assert list(sent[1][0].points[0].positions) == [1.5] * 6
    finally:
        _teardown_node(node)


def test_send_goal_floors_duration(monkeypatch):
    """The real _send_goal path stretches 0.2 s teleop points to the minimum."""

    class _NeverFuture:
        def add_done_callback(self, cb):
            pass

    class _AcceptHandle:
        accepted = True

        def get_result_async(self):
            return _NeverFuture()

    class _AcceptFuture:
        def add_done_callback(self, cb):
            cb(self)

        def result(self):
            return _AcceptHandle()

    import rclpy

    monkeypatch.setattr(ActorController, '_wait_for_server', lambda self, t: True)
    rclpy.init()
    try:
        node = ActorController()
        captured = {}

        def fake_send(goal):
            captured['goal'] = goal
            return _AcceptFuture()

        monkeypatch.setattr(node._client, 'send_goal_async', fake_send)
        cmd = _make_cmd([[0.1] * 6])
        cmd.points[0].time_from_start.sec = 0
        cmd.points[0].time_from_start.nanosec = 200000000
        cmd.header.stamp.sec = 1234
        cmd.header.stamp.nanosec = 5678
        node._send_goal(cmd, is_init=False)
        sent_traj = captured['goal'].trajectory
        dur = sent_traj.points[0].time_from_start
        assert dur.sec + dur.nanosec * 1e-9 == pytest.approx(
            node._min_duration
        )
        # outgoing goal carries a zero stamp (controller start-now convention)
        assert sent_traj.header.stamp.sec == 0
        assert sent_traj.header.stamp.nanosec == 0
        # ...while the stored command keeps its local stamp for introspection
        assert cmd.header.stamp.sec == 1234
        assert cmd.header.stamp.nanosec == 5678
        assert node._goal_outstanding
    finally:
        _teardown_node(node)


def test_actor_params_yaml_valid():
    import pathlib

    cfg = pathlib.Path(__file__).resolve().parent.parent / 'config' / 'actor_params.yaml'
    data = yaml.safe_load(cfg.read_text())
    ctrl = data['actor_controller']['ros__parameters']
    assert ctrl['actor_command_topic'] == '/actor/joint_cmd'
    assert (
        ctrl['actor_follow_joint_trajectory_action']
        == '/actor_raw/niryo_robot_follow_joint_trajectory_controller'
        '/follow_joint_trajectory'
    )
    assert ctrl['joint_names'] == NED2_JOINT_NAMES
    assert float(ctrl['actor_send_rate_hz']) > 0
    assert float(ctrl['command_stale_timeout_sec']) > 0
    assert float(ctrl['initial_sync_duration_sec']) > 0
    assert float(ctrl['initial_sync_tolerance_rad']) > 0
    assert float(ctrl['goal_result_timeout_sec']) > 0
    assert float(ctrl['min_goal_duration_sec']) > 0
    relay = data['actor_joint_state_publisher']['ros__parameters']
    assert relay['actor_state_input_topic'] == '/actor_raw/joint_states'
    assert relay['actor_state_output_topic'] == '/actor/joint_states'
