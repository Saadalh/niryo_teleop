# AGENTS.md — development rules for coding agents

This file governs every change in this repository, for any coding agent
(including automated ones). Read it before editing code. If a request conflicts
with these rules, surface the conflict instead of silently breaking them.

## 1. Project scope

This repository provides the robot-side teleoperation stack for collecting VLA
datasets:

- a ROS 2 runtime that teleoperates a Niryo Ned2 actor with a Ned2 master;
- a LeRobot integration that records demonstrations as LeRobotDataset v3.0;
- no policy training and no simulation.

VLA training and deployment live in the separate `vlas` project. This
repository must remain self-contained and submodule-friendly.

## 2. Repository map

| Path | Purpose |
|---|---|
| `src/niryo_teleop/` | ROS 2 ament package: nodes, launch files, configs, tests |
| `src/niryo_teleop/niryo_teleop/master/` | master joint-state relay |
| `src/niryo_teleop/niryo_teleop/teleop/` | joint mapping controller |
| `src/niryo_teleop/niryo_teleop/actor/` | actor command sender and state relay |
| `lerobot_plugin/` | LeRobot `Robot` / `Teleoperator` integration package |
| `scripts/` | hardware-free mock driver and environment checks |
| `tests/` | workspace-level smoke tests |
| `docs/` | setup, collection, integration, and troubleshooting documents |

## 3. Architecture boundaries

- The ROS 2 package must not depend on `lerobot`, `torch`, or policy code.
- The LeRobot plugin must not import ROS node implementations; it communicates
  with the runtime only through the public topics.
- Training, evaluation, GPU use, and dataset storage belong to `vlas`.
- Do not add simulation or digital-twin code.
- During LeRobot-driven recording, `/actor/joint_cmd` has exactly one publisher:
  either the ROS `teleoperation_controller` or the LeRobot follower, never both.

## 4. LeRobot-first rule

Use supported LeRobot capabilities before writing custom code:

- `lerobot-record` for episode capture, video encoding, and dataset metadata;
- `LeRobotDataset` for reading, inspecting, and validating datasets;
- `Robot` and `Teleoperator` base classes plus `RobotConfig` /
  `TeleoperatorConfig` registration for this hardware;
- LeRobot `opencv` camera configs for USB cameras;
- LeRobot processors for observation/action transforms;
- `lerobot-dataset-viz`, `lerobot-replay`, and existing training workflows.

Never implement a custom dataset format, writer, video encoder, statistics
module, or normalization scheme. Add only the hardware adapter and workspace
orchestration that LeRobot does not provide. Do not modify the vendored
`vlas/lerobot/` checkout.

## 5. vlas integration requirements (mandatory)

Every change must remain valid when this repository is a submodule of `vlas`:

- No absolute paths in code, launch files, configs, or tests.
- Do not assume a directory name, parent layout, or current working directory;
  resolve assets through package share directories or repository-relative paths.
- Keep the public interface stable: topics, message types, launch files,
  executable names, and the `lerobot_robot_niryo` plugin identifiers.
- Any breaking interface change requires updating the README,
  `docs/VLAS_INTEGRATION.md`, tests, and the known `vlas` consumers together.
- Keep the package installable as an editable Python distribution without
  network access at import time.
- No import-time side effects: no hardware connections, ROS node creation,
  downloads, or file writes on import.
- Do not write datasets, checkpoints, logs, or caches into the repository;
  dataset roots are always explicit command-line or config values.
- Record meaningful versions/tags so `vlas` can pin the submodule.

## 6. Configuration and interfaces

- All robot IPs, namespaces, topics, rates, limits, and thresholds belong in
  YAML config, never in Python literals.
- Standard ROS messages are preferred over custom message types.
- Publish measured state from measurement paths and desired state from command
  paths; never blur the two.
- Keep one responsibility per node and one entry point per executable.
- Preserve joint ordering and names (`joint_1 … joint_6`) at every boundary.
- Fail loudly on invalid parameters at startup; never silently clamp or guess.

## 7. Physical safety

- The actor arm can move real hardware. Treat every change as safety-relevant.
- Never remove or weaken stale-state, timeout, limit, or shutdown protections.
- Keep motions small and slow by default; retain the initial actor sync.
- Do not replay stale commands after a dropout.
- Do not add automatic motion on startup, connect, or import.
- Document any change to limits, rates, or timing in the same change.

## 8. Coding standards

- Python 3.12, ROS 2 Jazzy conventions, `ament_python` layout.
- Follow the existing style: type hints where present, descriptive names,
  small functions, no dead code.
- Respect `line-length = 100` and the existing package/config naming.
- Keep changes minimal and focused; do not refactor unrelated code.
- Preserve existing behavior and public interfaces unless the task explicitly
  changes them.
- Do not reformat whole files or rewrite history.

## 9. Testing and verification

Every change must pass:

1. `colcon build` from the workspace root.
2. `.venv/bin/python -m pytest -q` (existing suite).
3. The no-hardware flow from the README: both topics appear, message types are
   correct, and rates are sane.
4. Any new test must run without hardware and without network access.

For data-collection work, additionally:

5. Write a small dataset, reopen it with `LeRobotDataset`, verify features,
   shapes, episode/frame counts, and delete the test dataset.
6. Confirm no stale or `NaN` samples and that the recorded action equals the
   command actually issued.

Never claim a change works without running the relevant checks. If hardware is
required and unavailable, state explicitly what remains unverified.

## 10. Documentation

- Update `README.md` for user-visible behavior.
- Update `docs/DATA_COLLECTION.md` for collection behavior.
- Update `docs/VLAS_INTEGRATION.md` for interface or integration changes.
- Update `docs/TROUBLESHOOTING.md` for new failure modes.
- Keep the README short: introduction, features, setup, run commands, future
  plan. Technical depth belongs in `docs/`.
- Record installed dependency versions when they affect reproducibility.

## 11. Current status

Implemented: master state relay, identity teleoperation controller, actor
controller with sync and stale safety, dual-robot runtime, mock-driver flow,
unit tests.

Planned: LeRobot plugin implementation, camera observations, dataset recording
and validation, `vlas` training hand-off, policy rollout.

Do not implement planned features unless the task explicitly asks for them.
