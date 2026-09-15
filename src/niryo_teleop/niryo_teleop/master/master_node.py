"""Master node skeleton (Chunk 2).

Deprecated for production use: Chunk 3's master_joint_state_publisher is the
real measured-state relay. Kept as a no-hardware smoke-test node.
"""

import rclpy
from rclpy.node import Node


class MasterNode(Node):
    def __init__(self) -> None:
        super().__init__('master_node')
        self.declare_parameter('joint_states_topic', '/master/joint_states')
        self.declare_parameter('heartbeat_rate_hz', 1.0)
        rate = float(self.get_parameter('heartbeat_rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._on_heartbeat)
        self.get_logger().info(
            f"master_node started (topic={self.get_parameter('joint_states_topic').value})"
        )

    def _on_heartbeat(self) -> None:
        self.get_logger().info('master_node heartbeat (no hardware yet)', throttle_duration_sec=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
