# Troubleshooting

Known issues and fixes for the teleoperation stack. Start with
[`PHYSICAL_SETUP.md`](PHYSICAL_SETUP.md) for the expected hardware flow.

## Driver and startup

| Symptom | Cause → fix |
|---|---|
| Driver dies at import: `TypeError: unsupported operand ... InterfaceClass` (`autobahn/twisted/util.py: peer2str`) | System `zope.interface` 6.1 shadows the venv's newer version through `--system-site-packages`. In the driver venv run `.venv/bin/python -m pip install "autobahn==23.6.2"`, rebuild, and relaunch with that venv activated. |
| `Failed to create driver: No module named 'control_msgs'` | Missing runtime message packages → `sudo apt install -y ros-jazzy-control-msgs ros-jazzy-controller-manager-msgs ros-jazzy-moveit-msgs`, then relaunch. |
| `Failed to create driver: Timeout to connect to ROSBridge at ws://...:9090` | Robot off, cable unplugged, wrong IP, or rosbridge down → `ping -c 3 <robot>`, check TCP 9090, power and Ethernet, retry (a healthy driver takes ~12 s to initialise). |
| No `/master_raw/joint_states` or `/actor_raw/joint_states` | Driver misconfigured or robot unreachable → re-check IPs, `ping` both robots, rosbridge port 9090, and the driver logs. |
| Topics are missing although the nodes start cleanly | `install/` is stale after a `git pull` → re-run `colcon build && source install/setup.bash`. `ros2 launch` never reads `src/` directly. |

## Relay and controller

| Symptom | Cause → fix |
|---|---|
| `no joint_states received yet on …` | `input_topic` does not match the driver's (namespaced) topic → fix `master_params.yaml`, rebuild. |
| `dropping invalid master state (unexpected joint names …)` | Wrong robot model or namespace collision → verify the six joints `joint_1..6` with `ros2 topic echo --once`. |
| `/master/joint_states` flat while the arm moves | Master is not in learning/compliant mode → enable it in NiryoStudio. |
| `/actor/joint_cmd` silent but master state flows | State is too old (`maximum_state_age_sec`) or there is an RX gap (`stale_timeout_sec`) → check clocks and rates, or raise the thresholds slightly. |
| Jittery rates over Wi-Fi | Link jitter → use wired Ethernet and keep only the needed topics in the driver whitelist. |
| Node aborts: `param joint_scale has length …` | Vector parameter length is not 6 → fix `teleop_params.yaml`. |

## Actor arm

| Symptom | Cause → fix |
|---|---|
| Actor stuck in `INITIALIZING` | Action server, command topic, or measured state missing → `ros2 action list`, confirm `/actor/joint_cmd` flows, and make sure the actor is stiff (learning OFF). Check the actor log for the reason. |
| Live goals finish with status 6 / driver `EXECUTING→CANCELED` traceback | Overlapping goals are preempted by the robot, and the driver bridge cannot handle preemption → keep serialized goals; never stream faster than goal completion. |
| Goals ABORTED (status 6), while a single manual goal succeeds | Goal content is rejected by the Niryo controller: horizons below `min_goal_duration_sec` (default 1.0 s), a first point at `t=0`, or a workstation-stamped header (epoch 0 means "start now") → keep the duration floor, use single-point sync moves, zero the outgoing goal stamp. |
| Actor goal rejected or aborted | Controller limits or server state → compare limits with the target and check the action-server logs. The actor stays gated and retries toward the newest target. |

Site-specific values to fill in on the day: robot hostnames/IPs and driver
namespaces (hence `input_topic`). Everything else ships with working defaults.

## Dataset collection

| Symptom | Cause → fix |
|---|---|
| `symbol lookup error: …libcontrol_msgs…undefined symbol …fastcdr…Cdr9serializeEj` as soon as a goal is sent | Installed `ros-jazzy-control-msgs` was built against a newer Fast-CDR than `ros-jazzy-fastcdr` 2.2.5. Upgrade the pair: `sudo apt update && sudo apt install --only-upgrade ros-jazzy-fastcdr ros-jazzy-control-msgs`, then re-open the terminals. Verify with `python3 -c "from rclpy.serialization import serialize_message; from control_msgs.action import FollowJointTrajectory as A; serialize_message(A.Goal())"`. |
| `actor_controller gated: call the "enable" service…` in the log | Expected with `collection.launch.py`; the controller stays safe until `niryo-collect` reaches the ready prompt. To drive it manually: `ros2 service call /actor_controller/enable std_srvs/srv/Trigger`. |
| `enable service not available` | Runtime not launched (or launched from a stale `install/`) → `--no-runtime` only when the logic nodes are already running; otherwise rebuild and relaunch. |
| Collector times out with `actor did not reach LIVE_TELEOP` | The sync needs a fresh, steady master pose. Keep the master still, make sure `/actor/joint_cmd` is being published (the collector's bridge does this), and confirm the driver's action server: `ros2 action list | grep follow_joint_trajectory`. |
| `no fresh master state on /master/joint_states` / `no fresh actor state …` | Driver or relay not running, wrong namespace, or actor state relay missing → check `ros2 topic hz` for both `_raw` and relayed topics. |
| `Rerun preview unavailable … falling back to OpenCV` | `rerun-sdk` is not installed in the active environment → `uv pip install rerun-sdk`, or set `camera.display_mode: opencv` / record with `--no-camera`. |
| Camera connect fails / `--no-camera` used intentionally | `/dev/video*` missing or busy (another process holds it) → `ls -l /dev/video*`, close other camera apps, fix `camera.index_or_path`. |
| `pyniryo` import installs OpenCV 5.x and LeRobot cameras break | `pyniryo` declares an unpinned OpenCV dependency → install `pyniryo` together with `"opencv-python==4.13.0.92"` in the same command so the pin wins. |
| Episodes are recorded but the dataset name already exists | The collector appends `-2`, `-3`, … instead of overwriting; check `datasets/` before assuming a failed save. |
| Ctrl-X does nothing | It is only accepted *between* episodes, while the Enter prompt is shown. Press Enter to stop the current episode first. |
