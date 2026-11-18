# vlas integration contract

This repository must be usable both standalone and as a **git submodule** of the
`vlas` workspace. Every change must preserve both modes.

## Ownership boundaries

| Layer | Lives here | Notes |
|---|---|---|
| ROS 2 teleoperation runtime | `src/niryo_teleop/` | nodes, launch files, parameters, tests |
| LeRobot integration | `lerobot_plugin/` | `Robot` / `Teleoperator` adapters, no policy code |
| Dataset storage | `vlas/datasets/` | never inside this repository |
| VLA training, evaluation, rollout workflows | `vlas/src/` | not part of this repository |

`vlas` keeps its own virtual environment and its vendored `lerobot/` checkout.
This repository never edits or vendors LeRobot sources.

## Installation inside vlas

Add the repository as a submodule, then install the plugin into the shared
environment:

```bash
cd /path/to/vlas
git submodule add <niryo_teleop-repo-url> hardware/niryo_teleop
uv pip install -e hardware/niryo_teleop/lerobot_plugin
```

The ROS runtime is still built with `colcon` in the submodule directory:

```bash
cd hardware/niryo_teleop
source /opt/ros/jazzy/setup.bash
source <vlas>/.venv/bin/activate
colcon build
source install/setup.bash
```

The exact submodule path is a `vlas` decision; nothing in this repository may
depend on it.

## Path independence rules

- No absolute paths in code, launch files, configs, or tests.
- Resolve package assets with `FindPackageShare`, `PathJoinSubstitution`, or
  paths relative to the repository root.
- Dataset roots, driver config paths, and camera devices come from CLI flags or
  environment variables, never from hard-coded locations.
- Do not read or write outside the repository unless the user explicitly points
  there (for example a dataset root).
- `ros2 launch` must work from any current working directory.

## Public interface

Stable, versioned surface that `vlas` may rely on:

| Kind | Name |
|---|---|
| Topic | `/master/joint_states` (`sensor_msgs/msg/JointState`) |
| Topic | `/actor/joint_cmd` (`trajectory_msgs/msg/JointTrajectory`) |
| Topic | `/actor/joint_states` (`sensor_msgs/msg/JointState`) |
| Launch | `niryo_teleop master.launch.py` |
| Launch | `niryo_teleop teleop.launch.py` |
| Launch | `niryo_teleop full_teleop.launch.py` |
| Plugin | `lerobot_robot_niryo` with `niryo_ned2_follower` / `niryo_ned2_leader` |
| CLI | `lerobot-record` with the plugin types |

Changing a topic name, message type, launch argument, or plugin identifier is a
breaking change: update this document, the README, and the `vlas` workflows in
the same change.

## Environment rules

- ROS 2 Jazzy and Python 3.12 are required.
- The ROS runtime uses the project virtual environment with
  `--system-site-packages`.
- The LeRobot plugin runs in the `vlas` environment, which provides `rclpy`,
  ROS message packages, LeRobot, and OpenCV.
- Do not add `lerobot`, `torch`, or policy dependencies to the ROS package.
- Pin versions that matter (LeRobot, ROS distro, Python) and record them in the
  README or this document.

## Dataset rules

- Format is always LeRobotDataset v3.0, written by `lerobot-record`.
- Dataset roots live under `vlas/datasets/` and are passed explicitly.
- `repo_id` uses the `local/…` namespace unless the user requests a Hub push.
- Recorded data is never committed to either repository.

## Verification before integrating

1. `colcon build` succeeds and the existing pytest suite is green.
2. The no-hardware flow produces both teleoperation topics.
3. `pip show lerobot_robot_niryo` resolves to the submodule path.
4. `lerobot-record --help` lists the plugin types without import errors.
5. A tiny dataset is written, reopened with `LeRobotDataset`, and deleted.
6. `git status` in both repositories shows no generated artifacts.
