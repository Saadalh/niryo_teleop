#!/usr/bin/env python3
"""Mock Ned2 driver joint_states publisher (TEST TOOLING ONLY).

Emulates what the official ned-ros2-driver bridges from a real Ned2:
sensor_msgs/JointState on the master's input topic with Ned2 joint names,
radians, joint_1..joint_6 ordering. Used to verify
master_joint_state_publisher without hardware.

NEVER use against a real robot; launch the real ned-ros2-driver instead.

Usage (each in its own terminal, workspace sourced):
    python3 scripts/mock_ned2_driver.py --topic /master_raw/joint_states --rate 50 --mode sine
    ros2 run niryo_teleop master_joint_state_publisher
    ros2 topic echo /master/joint_states
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = [f'joint_{i}' for i in range(1, 7)]
BASE_POSE = [0.2, -0.4, 0.3, 0.0, 0.6, -0.3]
AMPLITUDE = [0.4, 0.25, 0.3, 0.5, 0.3, 0.6]


class MockDriver(Node):
    def __init__(self, topic: str, rate: float, mode: str):
        super().__init__('mock_ned2_driver')
        self._pub = self.create_publisher(JointState, topic, 10)
        self._mode = mode
        self._t0 = time.monotonic()
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'mock driver publishing on {topic} @ {rate} Hz (mode={mode})'
        )

    def _tick(self):
        t = time.monotonic() - self._t0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = list(JOINT_NAMES)
        if self._mode == 'sine':
            msg.position = [
                b + a * math.sin(2 * math.pi * 0.2 * t + i)
                for i, (b, a) in enumerate(zip(BASE_POSE, AMPLITUDE))
            ]
            msg.velocity = [a * 2 * math.pi * 0.2 * math.cos(2 * math.pi * 0.2 * t + i)
                            for i, a in enumerate(AMPLITUDE)]
            msg.effort = [0.0] * 6
        else:  # static
            msg.position = list(BASE_POSE)
        self._pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/master_raw/joint_states')
    ap.add_argument('--rate', type=float, default=50.0)
    ap.add_argument('--mode', choices=['sine', 'static'], default='sine')
    args = ap.parse_args()
    rclpy.init()
    node = MockDriver(args.topic, args.rate, args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
