"""Actor joint-state publisher: measured actor state relay.

Pipeline::

    Physical Ned2 actor
        -> ned-ros2-driver publishes sensor_msgs/JointState on <input_topic>
           (default /actor_raw/joint_states)
        -> ActorJointStatePublisher (this node, validated relay)
        -> /actor/joint_states (sensor_msgs/JointState, measured state)

Same principles as the master publisher: validate six Ned2 joints, preserve
frame_id and joint ordering, forward velocity/effort only when valid,
re-stamp with the workstation ROS 2 clock (robot clock not trusted), republish
every valid message immediately (no rate limiting), and warn on stale input
based on local reception time.
"""

from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from niryo_teleop.master.master_joint_state_publisher import (
    NED2_JOINT_NAMES,
    build_master_msg,
    validate_joint_state,
)


class ActorJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('actor_joint_state_publisher')
        self.declare_parameter('actor_state_input_topic', '/actor_raw/joint_states')
        self.declare_parameter('actor_state_output_topic', '/actor/joint_states')
        self.declare_parameter('joint_names', NED2_JOINT_NAMES)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('require_exact_joint_names', True)

        self._input_topic = str(self.get_parameter('actor_state_input_topic').value)
        self._output_topic = str(self.get_parameter('actor_state_output_topic').value)
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
            f'actor_joint_state_publisher: {self._input_topic} -> '
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

        out = build_master_msg(msg, self._joint_names, now.to_msg())
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
                '(is ned-ros2-driver running and connected to the actor Ned2?)',
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
    node = ActorJointStatePublisher()
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
