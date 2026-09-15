"""Colcon/pytest smoke tests for the skeleton package."""


def test_import_nodes():
    from niryo_teleop.master.master_node import MasterNode
    from niryo_teleop.teleop.teleop_controller_node import (
        NED2_JOINT_NAMES,
        TeleopControllerNode,
    )

    assert MasterNode is not None
    assert TeleopControllerNode is not None
    assert NED2_JOINT_NAMES == [
        'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6',
    ]
