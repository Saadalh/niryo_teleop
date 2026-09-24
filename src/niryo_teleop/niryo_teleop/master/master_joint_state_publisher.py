"""Master joint-state publisher (Chunk 3).

Pipeline::

    Physical Ned2 master
        -> ned-ros2-driver (official ROS 2 bridge, rosbridge) publishes
           sensor_msgs/JointState on <input_topic> (default /master_raw/joint_states)
        -> MasterJointStatePublisher (this node, validated relay)
        -> /master/joint_states (sensor_msgs/JointState, measured state)

Architecture note (Chunk 1/3 rule: do not duplicate the Niryo stack):
the driver's ``/master_raw/joint_states`` already *is* the measured master state, so this
node does NOT poll the robot over TCP (no pyniryo polling loop). It subscribes,
validates, and immediately republishes under the project namespace (no rate
limiting: every valid message is republished once). Timestamps are
workstation-local: the incoming Niryo header.stamp is NOT trusted (robot clock
unsynced), each output is re-stamped with this node's ROS 2 clock at relay
time; frame_id is preserved. The
physical connection (robot IP/hostname, rosbridge port, namespace) is handled
by ``ned-ros2-driver`` (see README hardware section); this node only needs the
driver's topic name.

Ned2 joint interface (documented, preserved end to end):
    names    = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    count    = 6
    units    = radians (positions; velocities rad/s, efforts driver units)
    ordering = joint_1 (base) .. joint_6 (tip); never reordered.
    velocity / effort are forwarded only when present with matching length;
    otherwise published empty (never fabricated).

Fail-safe behaviour:
    - invalid messages (bad sizes, NaN/Inf positions, wrong joint names when
      require_exact_joint_names=true) are dropped with throttled warnings;
    - input timeout (stale_timeout_sec without messages) logs throttled
      warnings (driver/robot/disconnect hint) and publishes nothing stale;
    - recovers automatically when the driver comes back;
    - clean shutdown via standard rclpy spin/destroy pattern.
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


NED2_JOINT_NAMES: List[str] = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6',
]


def _is_finite_list(values) -> bool:
    return all(math.isfinite(float(v)) for v in values)


def validate_joint_state(
    msg: JointState,
    expected_names: List[str],
    require_exact_names: bool = True,
) -> Tuple[bool, str]:
    """Check that *msg* carries usable measured positions.

    Returns (ok, reason). Velocity/effort problems do NOT fail validation;
    they are cleared at build time instead (never fabricated).
    """
    if len(msg.name) == 0:
        return False, 'empty name[]'
    if len(msg.position) == 0:
        return False, 'empty position[]'
    if len(msg.name) != len(msg.position):
        return False, (
            f'name/position size mismatch ({len(msg.name)} vs {len(msg.position)})'
        )
    if len(msg.name) != len(expected_names):
        return False, (
            f'joint count {len(msg.name)} != expected {len(expected_names)}'
        )
    if require_exact_names and list(msg.name) != list(expected_names):
        return False, f'unexpected joint names {list(msg.name)}'
    if not _is_finite_list(msg.position):
        return False, 'non-finite value in position[]'
    return True, 'ok'


def build_master_msg(msg: JointState, expected_names: List[str], stamp) -> JointState:
    """Build the /master/joint_states message from a validated driver message.

    Re-stamps with the workstation ROS 2 clock value supplied by the caller
    (the incoming Niryo header.stamp is not trusted). Preserves frame_id and
    joint ordering. Forwards velocity and effort only when their length
    matches; otherwise leaves them empty.
    Caller must run validate_joint_state() first.
    """
    out = JointState()
    out.header.frame_id = msg.header.frame_id
    out.header.stamp = stamp
    if list(msg.name) == list(expected_names):
        out.name = list(expected_names)
        out.position = list(msg.position)
    else:
        # Size already validated; keep incoming order, do not silently remap.
        out.name = list(msg.name)
        out.position = list(msg.position)
    if len(msg.velocity) == len(msg.position) and _is_finite_list(msg.velocity):
        out.velocity = list(msg.velocity)
    else:
        out.velocity = []
    if len(msg.effort) == len(msg.position) and _is_finite_list(msg.effort):
        out.effort = list(msg.effort)
    else:
        out.effort = []
    return out


class MasterJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('master_joint_state_publisher')
        self.declare_parameter('input_topic', '/master_raw/joint_states')
        self.declare_parameter('output_topic', '/master/joint_states')
        self.declare_parameter('joint_names', NED2_JOINT_NAMES)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('require_exact_joint_names', True)

        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._joint_names: List[str] = list(
            self.get_parameter('joint_names').value
        )
        self._stale_timeout_ns = int(
            float(self.get_parameter('stale_timeout_sec').value) * 1e9
        )
        depth = int(self.get_parameter('qos_depth').value)
        self._require_exact = bool(
            self.get_parameter('require_exact_joint_names').value
        )

        self._pub = self.create_publisher(JointState, self._output_topic, depth)
        self._sub = self.create_subscription(
            JointState, self._input_topic, self._on_joint_state, depth
        )
        check_period = min(
            float(self.get_parameter('stale_timeout_sec').value), 1.0
        )
        self._stale_timer = self.create_timer(check_period, self._check_stale)

        self._last_rx_time = None
        self._received_any = False
        self._was_stale = False
        self._published_count = 0
        self._dropped_count = 0
        self.get_logger().info(
            f'master_joint_state_publisher: {self._input_topic} -> '
            f'{self._output_topic} (immediate republish), '
            f'joints={self._joint_names}'
        )

    def _on_joint_state(self, msg: JointState) -> None:
        now = self.get_clock().now()
        self._last_rx_time = now
        if self._was_stale:
            self._was_stale = False
            self.get_logger().info('input joint_states flowing again')
        self._received_any = True

        ok, reason = validate_joint_state(
            msg, self._joint_names, self._require_exact
        )
        if not ok:
            self._dropped_count += 1
            self.get_logger().warn(
                f'dropping invalid JointState ({reason})',
                throttle_duration_sec=5.0,
            )
            return

        out = build_master_msg(msg, self._joint_names, self.get_clock().now().to_msg())
        self._pub.publish(out)
        self._published_count += 1
        if self._published_count == 1:
            self.get_logger().info(
                f'publishing measured state on {self._output_topic} '
                f'(names={out.name}, '
                f'velocity={"yes" if out.velocity else "no"}, '
                f'effort={"yes" if out.effort else "no"})'
            )

    def _check_stale(self) -> None:
        if not self._received_any or self._last_rx_time is None:
            self.get_logger().warn(
                f'no joint_states received yet on {self._input_topic} '
                '(is ned-ros2-driver running and connected to the master Ned2?)',
                throttle_duration_sec=10.0,
            )
            return
        staleness_ns = (
            self.get_clock().now() - self._last_rx_time
        ).nanoseconds
        if staleness_ns > self._stale_timeout_ns:
            self._was_stale = True
            self.get_logger().warn(
                f'joint_states stale for {staleness_ns / 1e9:.1f}s on '
                f'{self._input_topic} (driver/robot disconnect?); '
                'not publishing stale state',
                throttle_duration_sec=5.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MasterJointStatePublisher()
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
