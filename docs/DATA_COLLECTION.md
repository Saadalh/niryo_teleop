# Data collection (planned)

**Status: design only — nothing in this document is implemented yet.**

Goal: record human demonstrations from the master/actor Ned2 pair as a
**LeRobotDataset v3.0**, ready for VLA training in the `vlas` project. No custom
dataset writer is allowed: `lerobot-record` owns episodes, video encoding,
metadata, and the dataset layout.

## Runtime layout

During collection, LeRobot owns the control loop. Only the ROS relays and the
actor controller run; the ROS `teleoperation_controller` must **not** run,
because `/actor/joint_cmd` must have exactly one publisher.

```text
master Ned2 → driver → /master_raw/joint_states → relay → /master/joint_states
                                                              │
                                                    NiryoNed2Leader (Teleoperator)
                                                              │ action
USB camera → LeRobot opencv camera → lerobot-record → NiryoNed2Follower (Robot)
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

ROS nodes to run: `master_joint_state_publisher`, `actor_joint_state_publisher`,
`actor_controller`. LeRobot runs in the `vlas` virtual environment, which already
provides `rclpy`, the ROS message packages, LeRobot, and OpenCV.

## LeRobot plugin

Distribution: `lerobot_robot_niryo` (the `lerobot_robot_` prefix makes
`register_third_party_plugins()` import it automatically).

| Component | Responsibility |
|---|---|
| `config_niryo_ned2.py` | `RobotConfig` / `TeleoperatorConfig` subclasses, registered as `niryo_ned2_follower` and `niryo_ned2_leader` |
| `niryo_ned2_follower.py` | `Robot`: reads `/actor/joint_states`, publishes `/actor/joint_cmd`, exposes camera features |
| `niryo_ned2_leader.py` | `Teleoperator`: reads `/master/joint_states`, returns joint positions |
| `ros_io.py` | Shared rclpy helpers: node lifecycle, latest-sample subscriptions, timeouts, shutdown |

Rules:

- Cameras use LeRobot's `opencv` camera configs (direct `/dev/videoN`); no ROS
  camera node and no `cv_bridge`.
- The follower refuses to act on stale state, mirroring the existing ROS safety
  behavior.
- Joint limits and mapping live in configuration, not hard-coded in Python.
- Any safety clamping must be a LeRobot **action processor**, not hidden inside
  `send_action`, so the recorded `action` equals the command actually issued.

## Dataset schema

| Feature | Type | Source |
|---|---|---|
| `observation.state` | `float32[6]` | actor measured joint positions (rad), `/actor/joint_states` |
| `observation.images.front` | video `(H, W, 3)` | LeRobot opencv camera |
| `action` | `float32[6]` | mapped master targets published to `/actor/joint_cmd` |
| `task` | `string` | language instruction for the episode |

- `fps` matches the camera (default 30 Hz); the 50 Hz joint stream is sampled
  by holding the latest value, so no resampling writer is needed.
- `robot_type`: `niryo_ned2`; joint order `joint_1 … joint_6`.
- Identity mapping is the initial default; scale/offset remain configuration.

## Target command

Illustrative only, pending the adapter implementation:

```bash
lerobot-record \
  --robot.type=niryo_ned2_follower \
  --teleop.type=niryo_ned2_leader \
  --robot.cameras="{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
  --robot.id=niryo_actor \
  --teleop.id=niryo_master \
  --dataset.repo_id=local/niryo_pick_place \
  --dataset.root=/path/to/vlas/datasets/niryo_pick_place \
  --dataset.fps=30 \
  --dataset.num_episodes=20 \
  --dataset.single_task="Pick up the red cube" \
  --display_data=true
```

Dataset roots are always passed explicitly; when the repository is a `vlas`
submodule, records belong under `vlas/datasets/` and stay out of git.

## Episode conventions

- One task instruction per dataset; keep object/pose variations in separate
  datasets until the schema is stable.
- Start every episode from the same rest pose; keep the workspace clear.
- Record only successful demonstrations; discard failed episodes.
- Keep sessions short and consistent in camera placement and lighting.
- Name datasets `<task>_<variant>`; `lerobot-record` appends a timestamp.

## Validation checklist

1. `lerobot-dataset-viz --repo-id … --root …` opens the dataset.
2. Episode and frame counts match the session.
3. `observation.state` and `action` have shape `[6]` and stay inside Ned2 limits.
4. Video streams decode and are time-aligned with the joint data.
5. Replay one episode through the actor before training.
6. No `NaN`, no all-zero frames, no duplicated frames.

## Implementation order

1. Plugin skeleton and config registration.
2. ROS I/O helpers with stale-state handling.
3. Leader teleoperator and follower robot (no cameras).
4. Camera features and a first recorded episode with the mock driver.
5. Physical recording, QA, and replay.
6. Dataset hand-off to `vlas` training workflows.
