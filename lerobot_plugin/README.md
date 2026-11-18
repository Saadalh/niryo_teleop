# lerobot_robot_niryo

LeRobot `Robot` / `Teleoperator` adapters and the `niryo-collect` session for the
Niryo Ned2 teleoperation stack.

| Kind | Identifier | Notes |
|---|---|---|
| `Robot` | `niryo_ned2_follower` | actor: reads `/actor/joint_states`, publishes `/actor/joint_cmd`, owns the OpenCV camera |
| `Teleoperator` | `niryo_ned2_leader` | master: reads `/master/joint_states`, clamps to Ned2 limits |

The leader clamps before `record_loop` builds the frame, so the recorded
`action` equals the command actually issued. ROS runtime nodes stay separate
processes; these adapters only use the public topics.

## Layout

```text
lerobot_plugin/
├── pyproject.toml
├── tests/                       # plugin tests (need lerobot + rclpy)
└── lerobot_robot_niryo/
    ├── __init__.py              # imports the modules that register the adapters
    ├── config_niryo_ned2.py     # RobotConfig / TeleoperatorConfig subclasses
    ├── niryo_ned2_follower.py   # Robot adapter (actor)
    ├── niryo_ned2_leader.py     # Teleoperator adapter (master)
    ├── ros_io.py                # rclpy bridge, latest-sample reads, enable service
    ├── collect.py               # `niryo-collect` session state machine
    ├── calibration.py           # pyniryo `calibrate_auto` + learning mode
    ├── camera.py                # LeRobot OpenCV camera + preview (Rerun/OpenCV)
    ├── config.py                # collection YAML loader and path resolution
    ├── naming.py                # dataset naming/validation
    └── terminal.py              # Enter / Ctrl-X prompts
```

## Installation

Inside the `vlas` workspace, with its virtualenv active:

```bash
uv pip install -e /path/to/niryo_teleop/lerobot_plugin
uv pip install pyniryo "opencv-python==4.13.0.92"   # optional extras, pinned
```

`rclpy`, ROS message packages, LeRobot, and OpenCV are **not** PyPI
dependencies: they come from the ROS installation and the `vlas` environment.
`pyniryo` pulls OpenCV 5.x by default, which breaks LeRobot's OpenCV cameras, so
install both in one command to keep the pin.

Optional extras: `lerobot_robot_niryo[calibration]` (pyniryo) and
`lerobot_robot_niryo[viz]` (rerun-sdk).

## Usage

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash            # the colcon workspace with niryo_teleop
niryo-collect                        # --help for flags
```

`niryo-collect` asks for safety/environment confirmation, calibrates both arms
with pyniryo, starts the gated runtime and the camera preview, enables the actor
controller, and records one episode per Enter press. Ctrl-X between episodes
finalizes and names the dataset.

`lerobot-record` can also use the adapters directly, but then the runtime nodes
(including an ungated `actor_controller`) must be started by hand, and the
dataset schema/clamping rules in [`../docs/DATA_COLLECTION.md`](../docs/DATA_COLLECTION.md)
still apply.

## Tests

```bash
cd /path/to/niryo_teleop
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /path/to/vlas/.venv/bin/python -m pytest lerobot_plugin/tests -q
```

They need `lerobot` and `rclpy`, so they live outside the workspace
`testpaths`; `PYTEST_DISABLE_PLUGIN_AUTOLOAD` keeps ROS's pytest plugins out of
the `vlas` environment.
