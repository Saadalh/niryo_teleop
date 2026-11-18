"""Actor controller: forward /actor/joint_cmd to the Niryo FollowJointTrajectory action.

Pipeline::

    /actor/joint_cmd (trajectory_msgs/JointTrajectory, from teleoperation_controller)
        -> subscriber validates and stores the LATEST valid command only
        -> fixed-rate send loop (actor_send_rate_hz)
            -> INITIALIZING: move the actor once to the current master pose,
               then enter LIVE_TELEOP
            -> LIVE_TELEOP: stream the latest fresh command to the actor's
               official FollowJointTrajectory action server

No master->actor mapping here (teleoperation_controller already generates the
desired actor command). No filtering, planning, or IK. Outgoing goals carry a
zero trajectory stamp (the controller rejects workstation-stamped goals;
epoch-0 means start-now) and a floored horizon (sub-second goals are
rejected); the /actor/joint_cmd topic itself keeps local stamps.

Backlog policy: only the newest valid command is ever kept. Intermediate
commands are overwritten, never queued, so the actor always chases the latest
master state instead of replaying history.

Action-server strategy: at most ONE outstanding goal at a time. A new goal is
sent only after the previous one completes. Overlapping goals are forbidden
because the official driver bridge cannot handle preemption: when the robot
preempts an older goal, the bridge calls ``goal_handle.canceled()`` on a goal
that is still EXECUTING, which raises ``RCLError: invalid transition`` and the
goal is reported ABORTED (status 6) — the robot then never executes anything
to completion. Serializing goals avoids this path entirely. A result-timeout
watchdog re-arms the gate if a goal hangs.

Safety: nothing is sent until the action server is available, a fresh valid
command exists, and (for startup) a fresh measured actor state exists. If
commands go stale, sending stops; it resumes automatically from the latest
command when fresh data returns. Hardware e-stop remains the Niryo/bench
procedure (see docs/PHYSICAL_SETUP.md).
"""

import math
from typing import List, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from niryo_teleop.master.master_joint_state_publisher import (
    NED2_JOINT_NAMES,
    validate_joint_state,
)

# Minimum delay between initialization attempts (avoids hammering a server
# that keeps rejecting goals).
_INIT_RETRY_DELAY_SEC = 1.0


def validate_actor_command(
    msg: JointTrajectory,
    expected_names: Sequence[str],
) -> Tuple[bool, str]:
    """Check that *msg* is a usable actor command. Returns (ok, reason)."""
    if list(msg.joint_names) != list(expected_names):
        return False, f'unexpected joint names/order {list(msg.joint_names)}'
    if len(msg.points) == 0:
        return False, 'empty points[]'
    for i, point in enumerate(msg.points):
        if len(point.positions) != len(expected_names):
            return False, (
                f'point {i} position count {len(point.positions)} '
                f'!= expected {len(expected_names)}'
            )
        if not all(math.isfinite(float(v)) for v in point.positions):
            return False, f'non-finite value in point {i} positions[]'
    return True, 'ok'


def target_positions(msg: JointTrajectory) -> List[float]:
    """Final desired positions of a validated command (last point wins)."""
    return list(msg.points[-1].positions)


def max_abs_diff(a: Sequence[float], b: Sequence[float]) -> float:
    """Largest absolute per-joint difference between two poses."""
    return max(abs(x - y) for x, y in zip(a, b))


def build_init_trajectory(
    joint_names: Sequence[str],
    target_positions_: Sequence[float],
    duration_sec: float,
    stamp,
) -> JointTrajectory:
    """Single-point sync move: target pose at t=duration.

    Deliberately a single point with a multi-second horizon. A first point at
    t=0 is avoided (the Niryo controller aborts it), as are sub-second
    horizons: the only manually verified working shape is a single point with
    a multi-second duration.
    """
    traj = JointTrajectory()
    traj.header.stamp = stamp
    traj.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(target_positions_)
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int(
        (float(duration_sec) - int(duration_sec)) * 1e9
    )
    traj.points = [point]
    return traj


def apply_min_duration(
    msg: JointTrajectory, min_duration_sec: float
) -> JointTrajectory:
    """Copy *msg*, flooring the trajectory horizon to *min_duration_sec*.

    The Niryo controller aborts sub-second goals (a 0.2 s single point is
    rejected while a 2 s one executes). Point times are scaled proportionally
    so multi-point shapes keep their profile; degenerate zero-time
    trajectories get the full minimum on the last point.
    """
    out = JointTrajectory()
    # Fresh header copy (never alias the input: callers mutate out.header).
    out.header.frame_id = msg.header.frame_id
    out.header.stamp.sec = msg.header.stamp.sec
    out.header.stamp.nanosec = msg.header.stamp.nanosec
    out.joint_names = list(msg.joint_names)
    times = [
        p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
        for p in msg.points
    ]
    total = times[-1] if times else 0.0
    if total >= min_duration_sec:
        for point in msg.points:
            clone = JointTrajectoryPoint()
            clone.positions = list(point.positions)
            clone.velocities = list(point.velocities)
            clone.accelerations = list(point.accelerations)
            clone.effort = list(point.effort)
            clone.time_from_start = point.time_from_start
            out.points.append(clone)
        return out
    n = len(msg.points)
    for i, point in enumerate(msg.points):
        clone = JointTrajectoryPoint()
        clone.positions = list(point.positions)
        clone.velocities = list(point.velocities)
        clone.accelerations = list(point.accelerations)
        clone.effort = list(point.effort)
        if total > 0.0:
            scaled = times[i] * min_duration_sec / total
        else:
            scaled = min_duration_sec * (i + 1) / n
        clone.time_from_start.sec = int(scaled)
        clone.time_from_start.nanosec = int((scaled - int(scaled)) * 1e9)
        out.points.append(clone)
    return out


class ActorController(Node):
    """Forward latest teleop commands to the actor FollowJointTrajectory action."""

    def __init__(self) -> None:
        super().__init__('actor_controller')
        self.declare_parameter('actor_command_topic', '/actor/joint_cmd')
        self.declare_parameter('actor_measured_state_topic', '/actor/joint_states')
        self.declare_parameter(
            'actor_follow_joint_trajectory_action',
            '/actor_raw/niryo_robot_follow_joint_trajectory_controller'
            '/follow_joint_trajectory',
        )
        self.declare_parameter('joint_names', NED2_JOINT_NAMES)
        self.declare_parameter('actor_send_rate_hz', 10.0)
        self.declare_parameter('command_stale_timeout_sec', 0.5)
        self.declare_parameter('measured_state_stale_timeout_sec', 1.0)
        self.declare_parameter('initial_sync_duration_sec', 3.0)
        self.declare_parameter('initial_sync_tolerance_rad', 0.05)
        self.declare_parameter('motion_epsilon_rad', 0.002)
        self.declare_parameter('min_goal_duration_sec', 1.0)
        self.declare_parameter('action_server_timeout_sec', 5.0)
        self.declare_parameter('goal_result_timeout_sec', 5.0)
        self.declare_parameter('qos_depth', 10)
        # Collection pipelines start the controller gated so calibration and
        # bench preparation cannot trigger a sync move. Live teleoperation keeps
        # the historical auto-start behavior.
        self.declare_parameter('auto_start', True)
        self.declare_parameter(
            'status_topic', '/actor/controller_status'
        )

        self._joint_names: List[str] = list(self.get_parameter('joint_names').value)
        self._action_name = str(
            self.get_parameter('actor_follow_joint_trajectory_action').value
        )
        self._send_rate = float(self.get_parameter('actor_send_rate_hz').value)
        self._cmd_stale_timeout = float(
            self.get_parameter('command_stale_timeout_sec').value
        )
        self._measured_stale_timeout = float(
            self.get_parameter('measured_state_stale_timeout_sec').value
        )
        self._sync_duration = float(
            self.get_parameter('initial_sync_duration_sec').value
        )
        self._sync_tolerance = float(
            self.get_parameter('initial_sync_tolerance_rad').value
        )
        self._epsilon = float(self.get_parameter('motion_epsilon_rad').value)
        self._min_duration = float(
            self.get_parameter('min_goal_duration_sec').value
        )
        server_timeout = float(
            self.get_parameter('action_server_timeout_sec').value
        )
        self._goal_timeout = float(
            self.get_parameter('goal_result_timeout_sec').value
        )
        depth = int(self.get_parameter('qos_depth').value)
        self._auto_start = bool(self.get_parameter('auto_start').value)

        status_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            status_qos,
        )
        self._enable_srv = self.create_service(
            Trigger, '~/enable', self._on_enable
        )

        self._client: ActionClient = ActionClient(
            self, FollowJointTrajectory, self._action_name
        )
        self._sub_cmd = self.create_subscription(
            JointTrajectory,
            str(self.get_parameter('actor_command_topic').value),
            self._on_joint_cmd,
            depth,
        )
        self._sub_measured = self.create_subscription(
            JointState,
            str(self.get_parameter('actor_measured_state_topic').value),
            self._on_measured_state,
            depth,
        )
        self._timer = self.create_timer(1.0 / self._send_rate, self._tick)

        # Latest-only storage: callbacks overwrite, the loop consumes.
        self._latest_cmd = None
        self._latest_cmd_rx_sec = 0.0
        self._cmd_updated = False
        self._latest_measured = None
        self._latest_measured_rx_sec = 0.0
        self._last_sent_positions = None
        self._enabled = self._auto_start
        self._state = 'INITIALIZING' if self._enabled else 'WAITING_FOR_ENABLE'
        # Single outstanding goal shared by init + live paths. A new goal is
        # only sent once the previous one completes (or the watchdog fires),
        # so goals can never overlap and preempt each other on the robot.
        self._goal_outstanding = False
        self._outstanding_is_init = False
        self._goal_sent_sec = 0.0
        self._watchdog_fired = False
        self._last_init_attempt_sec = float('-inf')

        self.get_logger().info(
            f'waiting for action server {self._action_name} '
            f'(up to {server_timeout} s)'
        )
        if self._enabled:
            self._server_ready = self._wait_for_server(server_timeout)
        else:
            self._server_ready = False
            self.get_logger().info(
                'actor_controller gated: call the "enable" service to start '
                'the initial sync and live teleoperation'
            )
        if not self._server_ready and self._enabled:
            self.get_logger().error(
                f'action server {self._action_name} not available yet; '
                'actor stays in INITIALIZING until it appears'
            )
        self.get_logger().info(
            f'actor_controller: {self.get_parameter("actor_command_topic").value} -> '
            f'{self._action_name} @ {self._send_rate} Hz '
            f'(state={self._state})'
        )
        self._publish_status()

    def _publish_status(self) -> None:
        """Publish the controller state for collection orchestration."""
        msg = String()
        msg.data = self._state
        self._status_pub.publish(msg)

    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        self._state = state
        self._publish_status()

    def _on_enable(self, request, response):
        """Enable motion after calibration/bench preparation."""
        del request
        if self._enabled:
            response.success = True
            response.message = f'already enabled (state={self._state})'
            return response
        self._enabled = True
        self._set_state('INITIALIZING')
        self.get_logger().info(
            'actor_controller enabled; syncing the actor to the master pose'
        )
        response.success = True
        response.message = 'enabled'
        return response

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _wait_for_server(self, timeout_sec: float) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def _on_joint_cmd(self, msg: JointTrajectory) -> None:
        ok, reason = validate_actor_command(msg, self._joint_names)
        if not ok:
            self.get_logger().warn(
                f'dropping invalid actor command ({reason})',
                throttle_duration_sec=5.0,
            )
            return
        # Latest wins: overwrite, never queue.
        self._latest_cmd = msg
        self._latest_cmd_rx_sec = self._now_sec()
        self._cmd_updated = True

    def _on_measured_state(self, msg: JointState) -> None:
        ok, _ = validate_joint_state(msg, self._joint_names, True)
        if not ok:
            return
        self._latest_measured = msg
        self._latest_measured_rx_sec = self._now_sec()

    def _fresh(self, rx_sec: float, timeout_sec: float) -> bool:
        return (self._now_sec() - rx_sec) <= timeout_sec

    def _tick(self) -> None:
        if not self._enabled:
            return  # gated until calibration/bench preparation is complete
        self._check_goal_watchdog()
        if self._state == 'INITIALIZING':
            self._tick_initializing()
        else:
            self._tick_live()

    def _check_goal_watchdog(self) -> None:
        """Re-arm the send gate if a goal result never arrives."""
        if not self._goal_outstanding:
            return
        if self._now_sec() - self._goal_sent_sec <= self._goal_timeout:
            return
        if not self._watchdog_fired:
            self._watchdog_fired = True
            self.get_logger().error(
                'actor goal result timeout '
                f'({self._goal_timeout} s without a result); re-arming gate, '
                'will retry the latest command'
            )
        self._goal_outstanding = False
        self._outstanding_is_init = False

    def _tick_initializing(self) -> None:
        if not self._server_ready:
            self._server_ready = self._wait_for_server(timeout_sec=1.0)
            if not self._server_ready:
                self.get_logger().warn(
                    f'action server {self._action_name} unavailable; '
                    'actor stays in INITIALIZING',
                    throttle_duration_sec=10.0,
                )
                return
        if self._latest_cmd is None or not self._fresh(
            self._latest_cmd_rx_sec, self._cmd_stale_timeout
        ):
            self.get_logger().info(
                'INITIALIZING: waiting for a fresh /actor/joint_cmd',
                throttle_duration_sec=10.0,
            )
            return
        if self._latest_measured is None or not self._fresh(
            self._latest_measured_rx_sec, self._measured_stale_timeout
        ):
            self.get_logger().info(
                'INITIALIZING: waiting for fresh measured actor state',
                throttle_duration_sec=10.0,
            )
            return
        if self._goal_outstanding:
            return  # init goal still executing; result callback decides
        target = target_positions(self._latest_cmd)
        measured = list(self._latest_measured.position)
        if max_abs_diff(measured, target) <= self._sync_tolerance:
            self._last_sent_positions = list(target)
            self._set_state('LIVE_TELEOP')
            self.get_logger().info(
                'actor already at master pose; entering LIVE_TELEOP'
            )
            return
        if self._now_sec() - self._last_init_attempt_sec < _INIT_RETRY_DELAY_SEC:
            return
        self._last_init_attempt_sec = self._now_sec()
        self.get_logger().warn(
            'INITIALIZING: moving actor to current master pose '
            f'({self._sync_duration} s trajectory). Keep the master stationary '
            'and the actor workspace clear; live teleop starts after sync.',
            throttle_duration_sec=10.0,
        )
        sync = build_init_trajectory(
            self._joint_names,
            target,  # latest target at send time
            self._sync_duration,
            self.get_clock().now().to_msg(),
        )
        self._send_goal(sync, is_init=True)

    def _tick_live(self) -> None:
        if not self._cmd_updated:
            return  # nothing new since last send
        if not self._fresh(self._latest_cmd_rx_sec, self._cmd_stale_timeout):
            self._cmd_updated = False  # drop stale; never replay
            self.get_logger().warn(
                'actor commands stale; holding position (nothing sent)',
                throttle_duration_sec=5.0,
            )
            return
        if self._goal_outstanding:
            return  # previous goal still executing; latest is kept, not queued
        target = target_positions(self._latest_cmd)
        if self._last_sent_positions is not None and max_abs_diff(
            target, self._last_sent_positions
        ) <= self._epsilon:
            self._cmd_updated = False  # identical pose; skip resend
            return
        self._send_goal(self._latest_cmd, is_init=False)
        self._last_sent_positions = list(target)
        self._cmd_updated = False

    def _send_goal(self, trajectory: JointTrajectory, is_init: bool) -> None:
        goal = FollowJointTrajectory.Goal()
        # Floor the horizon: sub-second goals are rejected by the controller.
        floored = apply_min_duration(trajectory, self._min_duration)
        # Zero the stamp: the Niryo controller aborts workstation-stamped goals
        # (clock skew vs the robot), while an all-zero stamp — the shape of a
        # manually verified working goal — is treated as start-now. The stored
        # /actor/joint_cmd keeps its local stamp for introspection; only this
        # outgoing copy is altered.
        floored.header.stamp = Time()
        goal.trajectory = floored
        self._goal_outstanding = True
        self._outstanding_is_init = is_init
        self._goal_sent_sec = self._now_sec()
        self._watchdog_fired = False
        future = self._client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, init=is_init: self._on_goal_response(f, init)
        )

    def _on_goal_response(self, future, is_init: bool) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(
                f'actor FollowJointTrajectory goal {"(init sync) " if is_init else ""}'
                'rejected by action server'
            )
            self._goal_outstanding = False  # stay gated; tick retries
            self._outstanding_is_init = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, init=is_init: self._on_goal_result(f, init)
        )

    def _on_goal_result(self, future, is_init: bool) -> None:
        status = future.result().status
        self._goal_outstanding = False
        self._outstanding_is_init = False
        if not is_init:
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(
                    f'live actor goal finished with status {status}; '
                    'continuing with latest command',
                    throttle_duration_sec=5.0,
                )
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Next tick re-compares newest measured vs LATEST target: enters
            # LIVE_TELEOP only if still within tolerance (covers the master
            # having moved during the sync motion).
            self.get_logger().info('initialization move succeeded; re-checking sync')
        else:
            self.get_logger().error(
                f'initialization move failed (status {status}); '
                'actor stays in INITIALIZING and will retry toward the newest target'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Launch may already have shut the context down on SIGINT; guard
        # so Ctrl-C exits cleanly instead of raising RCLError noise.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
