# Data collection

Record human demonstrations from the master/actor Ned2 pair as a
**LeRobotDataset v3.0**, ready for VLA training in the `vlas` project. No custom
dataset writer is used: LeRobot's `record_loop`, `LeRobotDataset`, and video
encoding own the dataset.

## Session flow

`niryo-collect` (console script of `lerobot_robot_niryo`) drives one interactive
session:

1. **Safety prompt** — clear the workspace, keep the emergency stop at hand.
   Press Enter to confirm.
2. **Calibration** — `pyniryo` runs `calibrate_auto()` on the master, then the
   actor, and sets learning mode (master ON, actor OFF). Skip with
   `--skip-calibration`.
3. **Environment prompt** — power, cabling, camera, workspace. Press Enter.
   Leave the master still: the actor syncs to it right after this step.
4. **Runtime + preview** — the ROS relays and the *gated* actor controller
   start; the camera preview opens in its own window (Rerun, or OpenCV when
   Rerun is unavailable).
5. **Enable + sync** — the collector calls `/actor_controller/enable`, the
   controller moves the actor once to the current master pose, then reports
   `LIVE_TELEOP` on `/actor/controller_status`.
6. **Episodes** — between episodes the actor mirrors the master so the scene can
   be reset. Enter starts an episode, Enter stops and saves it, Enter starts the
   next one.
7. **Finish** — Ctrl-X *between* episodes finalizes the dataset and asks for a
   name (Enter keeps the default). The dataset path is printed at the end.

An episode also stops automatically after `collection.max_episode_seconds`.

Default name: `niryo-dataset-YY-MM-DD_HH-MM`. Existing directories are never
overwritten: a `-2`, `-3`, … suffix is appended.

## Runtime layout

During collection, only the relays and the actor controller run. The ROS
`teleoperation_controller` must **not** run: `/actor/joint_cmd` has exactly one
publisher, the LeRobot follower.

```text
master Ned2 → driver → /master_raw/joint_states → relay → /master/joint_states
                                                              │
                                                    NiryoNed2Leader (Teleoperator)
                                                              │ action (clamped)
camera ── LeRobot opencv camera ────────────────> NiryoNed2Follower (Robot)
                                                              │
                                                       /actor/joint_cmd
                                                              │
                                              actor_controller → FollowJointTrajectory
                                                              │
                                                         actor Ned2
actor Ned2 → driver → /actor_raw/joint_states → relay → /actor/joint_states
                                                              │
                                                    NiryoNed2Follower.get_observation()
```

`niryo_teleop/launch/collection.launch.py` starts the two relays and
`actor_controller` with `auto_start: false`. That gate is what keeps calibration
and bench preparation from moving the arm. The official
`niryo_ned_ros2_driver` is a separate process (see
[`PHYSICAL_SETUP.md`](PHYSICAL_SETUP.md)).

## LeRobot plugin

Distribution: `lerobot_robot_niryo`. Importing it registers:

| Component | Identifier | Responsibility |
|---|---|---|
| `Robot` | `niryo_ned2_follower` | reads `/actor/joint_states`, publishes `/actor/joint_cmd`, owns the OpenCV camera |
| `Teleoperator` | `niryo_ned2_leader` | reads `/master/joint_states`, clamps to the Ned2 limits |

The leader clamps **before** the action reaches `record_loop`, so the recorded
`action` is exactly the command published to the actor. The follower publishes
the action unchanged. Stale samples raise instead of replaying the last command.

## Dataset schema

| Feature | Type | Source |
|---|---|---|
| `observation.state` | `float32[6]` | actor measured joint positions (rad), `/actor/joint_states` |
| `observation.images.<camera>` | video `(H, W, 3)` | LeRobot opencv camera, feature name from `camera.name` |
| `action` | `float32[6]` | clamped master targets published to `/actor/joint_cmd` |
| `task` | `string` | one instruction per dataset (`collection.task`) |

- `robot_type`: `niryo_ned2`; joint order `joint_1 … joint_6`.
- `fps` matches the camera (default 30 Hz); the 50 Hz joint stream is sampled by
  holding the latest value.
- `repo_id` is `local/<dataset-name>`; the dataset is never pushed to the Hub.

## Configuration

All settings live in `src/niryo_teleop/config/collection_params.yaml`
(installed into the package share):

- `datasets_dir` — dataset root; relative paths resolve against the repository
  root. Default `datasets`, which is git-ignored.
- `default_name_prefix`, `task`, `fps`, `max_episode_seconds`,
  `max_episodes`, `stream_timeout_sec`, `enable_timeout_sec`.
- `camera` — enable flag, feature name, device, resolution, fps, preview mode.
- `robots` — IPs, calibration flag, learning modes.
- `topics` — master/actor states, actor command, controller status, enable
  service.
- `joints` — names and limits (shared with `teleop_params.yaml`).
- `teleop` — between-episode bridge rate and command horizon.

CLI overrides:

```bash
niryo-collect --config <yaml> --datasets-dir <dir> --task "<instruction>" \
              --max-episodes N --skip-calibration --no-runtime --no-camera
```

`--no-runtime` assumes the relays and controller are already running.
`--no-camera` records joint observations and actions only.

## Commands

```bash
# once per environment (vlas venv provides lerobot, rclpy, OpenCV)
uv pip install -e lerobot_plugin
uv pip install pyniryo "opencv-python==4.13.0.92"   # calibration, pinned

source /opt/ros/jazzy/setup.bash
source install/setup.bash
niryo-collect
```

The driver must already be publishing `/master_raw/joint_states` and
`/actor_raw/joint_states`.

## Episode conventions

- One task instruction per dataset; keep object/pose variations in separate
  datasets until the schema is stable.
- Start every episode from the same rest pose; keep the workspace clear.
- Record only successful demonstrations.
- Keep sessions short and consistent in camera placement and lighting.
- Name datasets `<task>_<variant>` when the automatic timestamp is not enough.

## Validation checklist

1. `lerobot-dataset-viz --repo-id local/<name> --root <datasets_dir>/<name>`
   opens the dataset.
2. Episode and frame counts match the session.
3. `observation.state` and `action` have shape `[6]` and stay inside Ned2
   limits.
4. Video streams decode and are time-aligned with the joint data.
5. No `NaN`, no all-zero frames, no duplicated frames.
6. `ros2 topic info /actor/joint_cmd` still lists one publisher and the
   controller status is `LIVE_TELEOP` in the dataset's time span.

## Verification status

Verified without hardware:

- plugin tests (`lerobot_plugin/tests`) for naming, config validation,
  calibration sequencing, clamping, and a full dataset round-trip through
  `record_loop` (schema, shapes, episode/frame counts);
- the gate: `collection.launch.py` leaves the actor in `WAITING_FOR_ENABLE`,
  `/actor/joint_cmd` has no publisher, and `/actor_controller/enable` moves the
  controller to `INITIALIZING`;
- the follower publishes exactly the clamped leader action to
  `/actor/joint_cmd`.

Still hardware-dependent: pyniryo calibration, camera capture, actor sync and
streaming on real arms, and the recorded video/state time alignment.

On this workstation the sync / `LIVE_TELEOP` path could not be exercised:
`ros-jazzy-control-msgs` crashes on a Fast-CDR symbol lookup
(`undefined symbol: …Cdr9serializeEj`) when a `FollowJointTrajectory` goal is
serialized. Upgrade the pair per
[`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) before the first physical session.

Plugin tests need the `vlas` environment (they require `lerobot`):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  <vlas>/.venv/bin/python -m pytest lerobot_plugin/tests -q
```
