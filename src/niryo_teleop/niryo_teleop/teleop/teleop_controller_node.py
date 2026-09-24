"""Teleoperation controller (Chunk 4).

Pipeline::

    /master/joint_states (sensor_msgs/JointState, measured master state)
        -> subscriber stores latest message (no processing here)
        -> fixed-rate control loop (control_rate_hz)
            -> validate_master_state(...)   count / names / order / finite / age
            -> map_master_to_actor(...)     q_cmd = scale * q_master + offset
            -> apply_joint_limits(...)      clamp to Ned2 limits (safe default)
            -> build_actor_command(...)     trajectory_msgs/JointTrajectory
            -> publish_actor_command(...)
        -> /actor/joint_cmd (ready for a future actor driver / ros2_control)

Default mapping is identity (scale=1, offset=0) for the identical-robot case.
Future extensions (scaling per task, inversion via negative scale, remapping,
filtering, velocity limiting, workspace constraints) slot in between the map
and limit stages without touching subscriber or publisher code.

JointTrajectory strategy: single-point streaming command. positions carry the
mapped+clamped command; velocities are left empty (the downstream
joint_trajectory_controller interpolates); time_from_start sets the execution
horizon for that point. header.stamp is the command (publish) time.

Stale-data safety: if no fresh master state arrives within stale_timeout_sec,
or the latest stamp is older than maximum_state_age_sec, the loop publishes
NOTHING (no replay of old motion) and logs throttled warnings. Recovers
automatically. Misconfigured vector params (wrong lengths) fail fast at
startup with a clear error instead of commanding wrong joints.
"""

import math
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


NED2_JOINT_NAMES: List[str] = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6',
]

# Ned2 command-validation limits, radians (ned_ros docs, Chunk 1).
NED2_LIMITS_MIN: List[float] = [
    -2.999871918328051,
    -1.8325957145941667,
    -1.3400637996813345,
    -2.090031779263347,
    -1.9200367101190883,
    -2.530029283691147,
]
NED2_LIMITS_MAX: List[float] = [
    2.999871918328051,
    0.6101671064972578,
    1.4922565104552499,
    2.090031779263347,
    1.9228292369222795,
    2.530029283691147,
]


def _stamp_to_sec(msg: JointState) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def validate_master_state(
    msg: JointState,
    expected_names: Sequence[str],
    max_age_sec: float,
    now_sec: float,
    rx_sec: float,
    require_exact_order: bool = True,
) -> Tuple[bool, str]:
    """Validate a received master state. Returns (ok, reason)."""
    expected = list(expected_names)
    if len(msg.name) == 0:
        return False, 'empty name[]'
    if len(msg.position) == 0:
        return False, 'empty position[]'
    if len(msg.name) != len(msg.position):
        return False, (
            f'name/position size mismatch ({len(msg.name)} vs {len(msg.position)})'
        )
    if len(msg.name) != len(expected):
        return False, (
            f'joint count {len(msg.name)} != expected {len(expected)}'
        )
    if require_exact_order and list(msg.name) != expected:
        return False, f'unexpected joint names/order {list(msg.name)}'
    if not all(math.isfinite(float(v)) for v in msg.position):
        return False, 'non-finite value in position[]'
    stamp_sec = _stamp_to_sec(msg)
    effective_sec = stamp_sec if stamp_sec > 0.0 else rx_sec
    if now_sec - effective_sec > max_age_sec:
        return False, (
            f'stale stamp (age {now_sec - effective_sec:.2f}s '
            f'> max {max_age_sec:.2f}s)'
        )
    return True, 'ok'


def map_master_to_actor(
    positions: Sequence[float],
    scale: Sequence[float],
    offsets: Sequence[float],
) -> List[float]:
    """Joint-space mapping: q_cmd[i] = scale[i] * q_master[i] + offsets[i]."""
    pos, scl, off = list(positions), list(scale), list(offsets)
    if not (len(pos) == len(scl) == len(off)):
        raise ValueError(
            f'mapping size mismatch: positions={len(pos)}, '
            f'scale={len(scl)}, offsets={len(off)}'
        )
    return [s * p + o for p, s, o in zip(pos, scl, off)]


def apply_joint_limits(
    positions: Sequence[float],
    limits_min: Sequence[float],
    limits_max: Sequence[float],
) -> Tuple[List[float], bool]:
    """Clamp positions into [min, max]. Returns (clamped, did_clamp)."""
    pos, lo, hi = list(positions), list(limits_min), list(limits_max)
    if not (len(pos) == len(lo) == len(hi)):
        raise ValueError(
            f'limit size mismatch: positions={len(pos)}, '
            f'min={len(lo)}, max={len(hi)}'
        )
    clamped = [min(max(p, l), h) for p, l, h in zip(pos, lo, hi)]
    return clamped, clamped != pos


def build_actor_command(
    joint_names: Sequence[str],
    positions: Sequence[float],
    stamp,
    time_from_start_sec: float,
) -> JointTrajectory:
    """Build the single-point /actor/joint_cmd trajectory message."""
    cmd = JointTrajectory()
    cmd.header.stamp = stamp
    cmd.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = int(time_from_start_sec)
    point.time_from_start.nanosec = int(
        (float(time_from_start_sec) - int(time_from_start_sec)) * 1e9
    )
    cmd.points = [point]
    return cmd


class TeleopControllerNode(Node):
    """Fixed-rate teleoperation controller (node name: teleoperation_controller)."""

    def __init__(self) -> None:
        super().__init__('teleoperation_controller')
        self.declare_parameter('master_topic', '/master/joint_states')
        self.declare_parameter('actor_topic', '/actor/joint_cmd')
        self.declare_parameter('joint_names', NED2_JOINT_NAMES)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('joint_scale', [1.0] * 6)
        self.declare_parameter('joint_offsets', [0.0] * 6)
        self.declare_parameter('joint_limits_min', NED2_LIMITS_MIN)
        self.declare_parameter('joint_limits_max', NED2_LIMITS_MAX)
        self.declare_parameter('maximum_state_age_sec', 0.5)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('time_from_start_sec', 0.2)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('require_exact_order', True)

        self._joint_names: List[str] = list(self.get_parameter('joint_names').value)
        self._scale: List[float] = [float(v) for v in self.get_parameter('joint_scale').value]
        self._offsets: List[float] = [float(v) for v in self.get_parameter('joint_offsets').value]
        self._limits_min: List[float] = [float(v) for v in self.get_parameter('joint_limits_min').value]
        self._limits_max: List[float] = [float(v) for v in self.get_parameter('joint_limits_max').value]
        n = len(self._joint_names)
        for label, vec in (
            ('joint_scale', self._scale),
            ('joint_offsets', self._offsets),
            ('joint_limits_min', self._limits_min),
            ('joint_limits_max', self._limits_max),
        ):
            if len(vec) != n:
                raise RuntimeError(
                    f'param {label} has length {len(vec)} but joint_names has {n}; '
                    'refusing to start (would command wrong joints)'
                )

        self._max_age = float(self.get_parameter('maximum_state_age_sec').value)
        self._stale_timeout = float(self.get_parameter('stale_timeout_sec').value)
        self._tfs = float(self.get_parameter('time_from_start_sec').value)
        self._require_exact = bool(self.get_parameter('require_exact_order').value)
        depth = int(self.get_parameter('qos_depth').value)
        rate = float(self.get_parameter('control_rate_hz').value)

        self._pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter('actor_topic').value), depth
        )
        self._sub = self.create_subscription(
            JointState,
            str(self.get_parameter('master_topic').value),
            self._on_master_state,
            depth,
        )
        self._timer = self.create_timer(1.0 / rate, self._control_tick)

        self._latest_msg = None
        self._latest_rx_sec = 0.0
        self._published_count = 0
        self.get_logger().info(
            f'teleoperation_controller: '
            f'{self.get_parameter("master_topic").value} -> '
            f'{self.get_parameter("actor_topic").value} @ {rate} Hz, '
            f'scale={self._scale}, offsets={self._offsets}'
        )

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _on_master_state(self, msg: JointState) -> None:
        # Reception only — all processing happens in the fixed-rate loop.
        self._latest_msg = msg
        self._latest_rx_sec = self._now_sec()

    def _control_tick(self) -> None:
        now_sec = self._now_sec()
        if self._latest_msg is None:
            self.get_logger().info(
                'waiting for master state on '
                f'{self.get_parameter("master_topic").value}',
                throttle_duration_sec=10.0,
            )
            return
        if now_sec - self._latest_rx_sec > self._stale_timeout:
            self.get_logger().warn(
                f'no fresh master state for {now_sec - self._latest_rx_sec:.1f}s '
                '(master/driver stopped?); holding last command: publishing nothing',
                throttle_duration_sec=5.0,
            )
            return
        ok, reason = validate_master_state(
            self._latest_msg,
            self._joint_names,
            self._max_age,
            now_sec,
            self._latest_rx_sec,
            self._require_exact,
        )
        if not ok:
            self.get_logger().warn(
                f'dropping invalid master state ({reason})',
                throttle_duration_sec=5.0,
            )
            if 'stale stamp' in reason:
                self._latest_msg = None  # invalid data; wait for fresh input
            return
        positions = list(self._latest_msg.position)
        if list(self._latest_msg.name) != self._joint_names:
            # Relaxed-order mode only (strict mode already rejected above).
            self.get_logger().warn(
                'master joint order differs from config; using received order',
                throttle_duration_sec=10.0,
            )
        commanded = map_master_to_actor(positions, self._scale, self._offsets)
        commanded, did_clamp = apply_joint_limits(
            commanded, self._limits_min, self._limits_max
        )
        if did_clamp:
            self.get_logger().warn(
                f'command clamped to joint limits: {commanded}',
                throttle_duration_sec=5.0,
            )
        cmd = build_actor_command(
            self._joint_names, commanded, self.get_clock().now().to_msg(), self._tfs
        )
        self.publish_actor_command(cmd)

    def publish_actor_command(self, cmd: JointTrajectory) -> None:
        self._pub.publish(cmd)
        self._published_count += 1
        if self._published_count == 1:
            self.get_logger().info(
                f'publishing actor commands on '
                f'{self.get_parameter("actor_topic").value} '
                f'(joints={cmd.joint_names})'
            )


# Backwards/forwards-compatible alias for the Chunk 4 node name.
TeleoperationController = TeleopControllerNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopControllerNode()
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
