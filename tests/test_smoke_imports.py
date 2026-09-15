"""Top-level venv smoke tests (run with .venv activated)."""


def test_package_importable():
    import niryo_teleop  # noqa: F401


def test_nodes_importable_without_rclpy_init():
    from niryo_teleop.master.master_node import MasterNode
    from niryo_teleop.teleop.teleop_controller_node import TeleopControllerNode

    assert MasterNode is not None
    assert TeleopControllerNode is not None
