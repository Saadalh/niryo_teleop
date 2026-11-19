# niryo_teleop — Ned2-to-Ned2 teleoperation for VLA data collection

## Introduction

Teleoperate a Niryo **Ned2** actor arm with a second Ned2 master arm, and
record the demonstrations for training a Vision-Language-Action (VLA) model.

The target workflow:

```text
human moves the master Ned2
        ↓
actor Ned2 mirrors the motion
        ↓
camera + joint states are recorded as a LeRobot dataset
        ↓
VLA policy (GR00T, SmolVLA, …) is trained in the vlas project
        ↓
trained policy controls the Ned2
```

Teleoperation works today. Dataset collection is the next phase, and VLA
training/deployment lives in the separate `vlas` project.

## Features

**Working today**

- Measured master joint states relayed to `/master/joint_states`.
- Identity joint mapping from master to `/actor/joint_cmd` at 50 Hz.
- Actor controller that drives the real arm through Niryo's
  `FollowJointTrajectory` action, including a startup sync and stale-command
  safety.
- Dual-robot operation with namespaced driver topics.
- LeRobot adapters (`niryo_ned2_follower` / `niryo_ned2_leader`) and
  prompt-driven dataset collection into LeRobotDataset v3.0 (joint state,
  action, USB camera video).
- Hardware-free test flow with a mock driver.
- Unit tests for relay, mapping, limits, actor safety, and the dataset schema.

**Next phase (not implemented yet)**

- Dataset QA, replay, and hand-off to `vlas` training.
- Language/task annotation variants beyond one instruction per dataset.

## Project structure

```text
niryo_teleop/                  # colcon workspace + uv project
├── src/niryo_teleop/          # ROS 2 package: master, teleop, actor
│   ├── niryo_teleop/          # node implementations
│   ├── launch/                # master, teleop, full_teleop
│   ├── config/                # all tunable parameters
│   └── test/                  # ROS-level unit tests
├── lerobot_plugin/            # LeRobot adapters + `niryo-collect`
├── scripts/                   # mock driver and environment checks
├── docs/                      # setup, collection, integration notes
└── tests/                     # workspace smoke tests
```

## Installation

Requirements: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, and `uv`.
Niryo's official `ned-ros2-driver` is needed only for physical robots.

```bash
cd ~/projects/niryo_teleop
uv venv --system-site-packages --python /usr/bin/python3.12
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
uv pip install colcon-common-extensions colcon-ros "pytest==7.4.4"
# LeRobot, pyniryo, and the pinned OpenCV come from the plugin's own
# dependencies; the first install downloads ~3 GB (torch). rclpy is not
# installed here: it comes from the sourced ROS 2 installation.
uv pip install -e lerobot_plugin
colcon build
source install/setup.bash
source .venv/bin/activate          # keep the env active in this shell
niryo-collect --help               # sanity check
```

That single `.venv` covers teleoperation tooling, the test suites, and dataset
collection, so no other project or virtualenv is required.

Re-run `colcon build && source install/setup.bash` after every `git pull` or
config/launch change. `ros2 run` and `ros2 launch` execute `install/`, not
`src/`.

The driver workspace and robot preparation are covered in
[`docs/PHYSICAL_SETUP.md`](docs/PHYSICAL_SETUP.md).

## Run commands

### Without hardware

Terminal 1 — mock master:

```bash
cd ~/projects/niryo_teleop
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 scripts/mock_ned2_driver.py --topic /master_raw/joint_states --rate 50 --mode sine
```

Terminal 2 — relay + controller:

```bash
cd ~/projects/niryo_teleop
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch niryo_teleop teleop.launch.py
```

Terminal 3 — inspect:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /master/joint_states /actor/joint_cmd
ros2 topic echo --field position /master/joint_states
```

### With hardware — master only

```bash
# Terminal 1: official driver (see PHYSICAL_SETUP.md for the driver workspace)
source /opt/ros/jazzy/setup.bash
source ~/projects/niryo_driver_ws/.venv/bin/activate
source ~/projects/niryo_driver_ws/install/setup.bash
ros2 launch niryo_ned_ros2_driver driver.launch.py \
  drivers_list_file:=$HOME/projects/niryo_driver_ws/src/ned-ros2-driver/niryo_ned_ros2_driver/config/master_driver.yaml

# Terminal 2: relay
cd ~/projects/niryo_teleop
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch niryo_teleop master.launch.py

# Terminal 3: move the master by hand (learning mode ON) and watch the values
ros2 topic hz /master_raw/joint_states /master/joint_states
ros2 topic echo --field position /master/joint_states
```

### With hardware — dual robot

```bash
# Terminal 1: driver for both robots
source /opt/ros/jazzy/setup.bash
source ~/projects/niryo_driver_ws/.venv/bin/activate
source ~/projects/niryo_driver_ws/install/setup.bash
ros2 launch niryo_ned_ros2_driver driver.launch.py \
  drivers_list_file:=$HOME/projects/niryo_teleop/src/niryo_teleop/config/dual_robot_drivers_list.yaml

# Terminal 2: full teleoperation chain
cd ~/projects/niryo_teleop
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch niryo_teleop full_teleop.launch.py

# Terminal 3: monitor
ros2 topic hz /master/joint_states /actor/joint_cmd /actor/joint_states
```

Keep the master still during the actor's initial sync (about 3 s), then move
slowly. Ctrl-C the teleop launch before the driver.

### Dataset collection (dual robot + camera)

The driver must already be publishing both `_raw` joint states. Then:

```bash
cd ~/projects/niryo_teleop
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source .venv/bin/activate          # lerobot, pyniryo, OpenCV (from the install step)
niryo-collect
```

The session asks for two confirmations (safety, environment), calibrates with
`pyniryo`, opens the camera preview, syncs the actor, then records one episode
per Enter press. Ctrl-X between episodes finishes and names the dataset.
Datasets go to `datasets/<name>` by default; edit
`src/niryo_teleop/config/collection_params.yaml` to change paths, camera, fps,
or limits.

## Future plan

1. **Dataset quality (next)** — episode/task conventions, validation, replay,
   and dataset summaries.
2. **VLA training (in `vlas`)** — fine-tune GR00T or SmolVLA on the recorded
   datasets.
3. **Policy deployment** — run a trained policy on the actor Ned2 with a safe
   command envelope and evaluation protocol.
4. **Integration** — keep this repository self-contained so it can be added to
   `vlas` as a submodule without path or packaging changes.

**Non-goals for now:** simulation/digital-twin integration, policy code in
this repository, and any custom dataset format. LeRobot is used wherever
possible.

See [`docs/DATA_COLLECTION.md`](docs/DATA_COLLECTION.md) for the collection
workflow and [`docs/VLAS_INTEGRATION.md`](docs/VLAS_INTEGRATION.md) for the
integration contract.

## Documentation

- [`docs/PHYSICAL_SETUP.md`](docs/PHYSICAL_SETUP.md) — robot prep, driver
  bridge, first physical session, safety.
- [`docs/DATA_COLLECTION.md`](docs/DATA_COLLECTION.md) — collection session,
  configuration, dataset schema.
- [`docs/VLAS_INTEGRATION.md`](docs/VLAS_INTEGRATION.md) — submodule and
  integration contract.
- [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) — known issues and fixes.
- [`AGENTS.md`](AGENTS.md) — development rules for coding agents.
- [`lerobot_plugin/README.md`](lerobot_plugin/README.md) — LeRobot plugin usage.
