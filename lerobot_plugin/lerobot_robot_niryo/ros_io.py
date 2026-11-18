"""Shared rclpy I/O for the Niryo LeRobot adapters.

The collector process owns one :class:`RosBridge` per adapter. Each bridge
creates a node and spins it in a background thread so LeRobot's synchronous
record loop can read the latest samples without blocking.

All freshness checks use ``time.monotonic()`` at reception time, never the
message stamp: the robots' clocks are not synchronised with the workstation.
"""
from __future__ import annotations

import threading
import time
from typing import List, Optional, Sequence

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RosBridge:
    """Latest-sample ROS I/O used by the leader and follower adapters."""

    def __init__(
        self,
        node_name: str,
        *,
        joint_names: Sequence[str],
        master_topic: str = "/master/joint_states",
        actor_state_topic: str = "/actor/joint_states",
        actor_cmd_topic: str = "/actor/joint_cmd",
        status_topic: str = "/actor/controller_status",
        enable_service: str = "/actor_controller/enable",
        qos_depth: int = 10,
        time_from_start_sec: float = 0.2,
    ) -> None:
        self.joint_names = list(joint_names)
        self._time_from_start = float(time_from_start_sec)
        self._lock = threading.Lock()
        self._master: Optional[List[float]] = None
        self._master_rx = 0.0
        self._actor: Optional[List[float]] = None
        self._actor_rx = 0.0
        self._status: Optional[str] = None

        self._node = Node(node_name)
        depth = int(qos_depth)
        self._sub_master = self._node.create_subscription(
            JointState, master_topic, self._on_master, depth
        )
        self._sub_actor = self._node.create_subscription(
            JointState, actor_state_topic, self._on_actor, depth
        )
        status_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._sub_status = self._node.create_subscription(
            String, status_topic, self._on_status, status_qos
        )
        self._pub_cmd = self._node.create_publisher(
            JointTrajectory, actor_cmd_topic, depth
        )
        self._enable_client = self._node.create_client(
            Trigger, enable_service
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._closed = False
        self._thread = threading.Thread(
            target=self._spin, name=f'{node_name}_spin', daemon=True
        )
        self._thread.start()

    # -- spinning ---------------------------------------------------------

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # executor shut down during close()
            pass

    # -- callbacks --------------------------------------------------------

    def _on_master(self, msg: JointState) -> None:
        if len(msg.position) != len(self.joint_names):
            return
        with self._lock:
            self._master = [float(v) for v in msg.position]
            self._master_rx = time.monotonic()

    def _on_actor(self, msg: JointState) -> None:
        if len(msg.position) != len(self.joint_names):
            return
        with self._lock:
            self._actor = [float(v) for v in msg.position]
            self._actor_rx = time.monotonic()

    def _on_status(self, msg: String) -> None:
        with self._lock:
            self._status = str(msg.data)

    # -- reads ------------------------------------------------------------

    def _latest(
        self, value: Optional[List[float]], rx_sec: float, stale_timeout_sec: float
    ) -> Optional[List[float]]:
        if value is None:
            return None
        if stale_timeout_sec > 0.0 and (
            time.monotonic() - rx_sec
        ) > stale_timeout_sec:
            return None
        return list(value)

    def master_positions(
        self, stale_timeout_sec: float = 0.5
    ) -> Optional[List[float]]:
        with self._lock:
            return self._latest(
                self._master, self._master_rx, stale_timeout_sec
            )

    def actor_positions(
        self, stale_timeout_sec: float = 0.5
    ) -> Optional[List[float]]:
        with self._lock:
            return self._latest(
                self._actor, self._actor_rx, stale_timeout_sec
            )

    def latest_status(self) -> Optional[str]:
        with self._lock:
            return self._status

    def wait_for_master(
        self, timeout_sec: float, stale_timeout_sec: float = 0.5
    ) -> bool:
        return self._wait_for(
            self.master_positions, timeout_sec, stale_timeout_sec
        )

    def wait_for_actor(
        self, timeout_sec: float, stale_timeout_sec: float = 0.5
    ) -> bool:
        return self._wait_for(
            self.actor_positions, timeout_sec, stale_timeout_sec
        )

    @staticmethod
    def _wait_for(read, timeout_sec: float, stale_timeout_sec: float) -> bool:
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if read(stale_timeout_sec) is not None:
                return True
            time.sleep(0.05)
        return False

    # -- writes -----------------------------------------------------------

    def publish_action(self, positions: Sequence[float]) -> None:
        """Publish a single-point streaming command for the actor."""
        if len(positions) != len(self.joint_names):
            raise ValueError(
                f'expected {len(self.joint_names)} positions, got {len(positions)}'
            )
        traj = JointTrajectory()
        traj.header.stamp = self._node.get_clock().now().to_msg()
        traj.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        sec = int(self._time_from_start)
        point.time_from_start = Duration(
            sec=sec, nanosec=int((self._time_from_start - sec) * 1e9)
        )
        traj.points = [point]
        self._pub_cmd.publish(traj)

    def enable_controller(self, timeout_sec: float = 5.0) -> tuple[bool, str]:
        """Call the actor controller's enable service."""
        if not self._enable_client.wait_for_service(timeout_sec=timeout_sec):
            return False, 'enable service not available'
        future = self._enable_client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_sec):
            return False, 'enable service timed out'
        try:
            response = future.result()
        except Exception as exc:  # service call failed
            return False, str(exc)
        return bool(response.success), str(response.message)

    # -- lifecycle --------------------------------------------------------

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        # The spin thread must leave spin() before the node is destroyed,
        # otherwise rclpy aborts on shutdown.
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
        except Exception:
            pass


def ensure_rclpy() -> None:
    """Initialise rclpy once for the collector process."""
    if not rclpy.ok():
        rclpy.init()


def shutdown_rclpy() -> None:
    if rclpy.ok():
        rclpy.shutdown()
