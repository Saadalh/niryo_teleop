"""pyniryo calibration sequencing (fake robot, no hardware)."""
import pytest

pytest.importorskip('lerobot')

from lerobot_robot_niryo.calibration import calibrate_robot, calibrate_robots  # noqa: E402


class _FakeRobot:
    def __init__(self, log, ip_address):
        self.log = log
        self.ip_address = ip_address
        self.learning_modes = []
        self.closed = False

    def calibrate_auto(self):
        self.log.append(('calibrate', self.ip_address))

    def set_learning_mode(self, enabled):
        self.log.append(('learning_mode', self.ip_address, bool(enabled)))

    def close_connection(self):
        self.closed = True
        self.log.append(('close', self.ip_address))


def _factory(log, created):
    def factory(ip_address):
        robot = _FakeRobot(log, ip_address)
        created.append(robot)
        return robot

    return factory


def test_calibrates_master_then_actor_with_learning_modes():
    log, created = [], []
    calibrate_robots(
        '10.0.0.1',
        '10.0.0.2',
        master_learning_mode=True,
        actor_learning_mode=False,
        robot_factory=_factory(log, created),
    )
    assert [entry[0] for entry in log] == [
        'calibrate',
        'learning_mode',
        'close',
        'calibrate',
        'learning_mode',
        'close',
    ]
    assert log[0][1] == '10.0.0.1'
    assert log[1] == ('learning_mode', '10.0.0.1', True)
    assert log[3][1] == '10.0.0.2'
    assert log[4] == ('learning_mode', '10.0.0.2', False)
    assert all(robot.closed for robot in created)


def test_connection_is_closed_when_calibration_fails():
    closed = []

    class _Failing(_FakeRobot):
        def calibrate_auto(self):
            raise RuntimeError('calibration failed')

        def close_connection(self):
            closed.append(self.ip_address)
            super().close_connection()

    def factory(ip_address):
        return _Failing([], ip_address)

    with pytest.raises(RuntimeError, match='calibration failed'):
        calibrate_robot('master', '10.0.0.1', learning_mode=True, robot_factory=factory)
    assert closed == ['10.0.0.1']


def test_dry_run_never_constructs_a_robot():
    def factory(ip_address):
        raise AssertionError(f'unexpected robot construction for {ip_address}')

    calibrate_robots(
        '10.0.0.1',
        '10.0.0.2',
        dry_run=True,
        robot_factory=factory,
    )
