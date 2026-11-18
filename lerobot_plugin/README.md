# lerobot_robot_niryo (scaffold)

LeRobot integration layer for the Niryo Ned2 teleoperation stack.

**Status: scaffold only.** No adapters are implemented yet. This directory
exists so the package layout and distribution name are fixed before development
starts.

## Target layout

```text
lerobot_plugin/
├── pyproject.toml
└── lerobot_robot_niryo/
    ├── __init__.py            # imports modules that perform registration
    ├── config_niryo_ned2.py   # RobotConfig / TeleoperatorConfig subclasses
    ├── niryo_ned2_follower.py # Robot: /actor/joint_states -> /actor/joint_cmd
    ├── niryo_ned2_leader.py   # Teleoperator: /master/joint_states
    └── ros_io.py              # shared rclpy helpers
```

## Planned registration

| Kind | Identifier |
|---|---|
| Robot | `niryo_ned2_follower` |
| Teleoperator | `niryo_ned2_leader` |

The distribution name starts with `lerobot_robot_`, so LeRobot's
`register_third_party_plugins()` imports it automatically.

## Planned installation

Inside the `vlas` workspace:

```bash
uv pip install -e path/to/niryo_teleop/lerobot_plugin
```

The environment must provide `rclpy`, the ROS 2 message packages, LeRobot, and
OpenCV. These are intentionally not declared as PyPI dependencies because they
come from the ROS installation and the `vlas` environment.

Design details and the target dataset schema:
[`../docs/DATA_COLLECTION.md`](../docs/DATA_COLLECTION.md).
