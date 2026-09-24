# Physical setup & getting started with Ned2 teleoperation

How to go from boxed robots to a working teleoperation pipeline:

```text
Master Ned2
  -> ned-ros2-driver (master_raw) => /master_raw/joint_states
  -> master_joint_state_publisher  => /master/joint_states
  -> teleoperation_controller      => /actor/joint_cmd
  -> actor_controller              => FollowJointTrajectory => Actor Ned2

Actor Ned2
  -> ned-ros2-driver (actor_raw)   => /actor_raw/joint_states
  -> actor_joint_state_publisher   => /actor/joint_states
```

First session goal: move the master by hand (learning/compliant mode) and
watch the actor arm follow. Start with very small, slow single-joint motions
and confirm `/actor/joint_states` tracks before anything larger.

## 1. What you need

- 2× Niryo **Ned2** arms with power supplies: one **master** (hand-guided),
  one **actor** (follows). Both are used from here on.
- Workstation: Ubuntu 24.04 + ROS 2 Jazzy (see `README.md` — this repo builds
  without touching the system install).
- Network: wired Ethernet strongly recommended (Wi-Fi/hotspot adds jitter to the
  rosbridge link). One switch/router, or direct Ethernet to each robot.
- NiryoStudio installed on the workstation (robot updates, calibration,
  learning-mode toggle). Download from Niryo's site if missing.
- Clear bench space: no obstacles in either arm's reach, emergency stop
  reachable (NiryoStudio stop / power switch).

## 2. Prepare both robots

1. **Place safely.** Both Ned2 arms firmly on the bench, clear workspace around
   the actor (it will move on its own), emergency stop reachable
   (NiryoStudio stop / power switch).
2. **Power and connect.** Power both arms, connect each to the workstation
   network (Ethernet preferred). Note both addresses: hostnames look like
   `ned2-aa-bbb-ccc.local`, or take the DHCP IPs from your router. Decide now
   which arm is **master** and which is **actor** and label them.
3. **Check reachability** from the workstation:
   ```bash
   ping -c 3 <master-ip>   # e.g. 169.254.200.200
   ping -c 3 <actor-ip>
   ```
4. **Update + calibrate in NiryoStudio.** Connect to each arm, install pending
   robot software updates, then run the motor calibration on **both**.
   Teleoperation on an uncalibrated arm is meaningless (joint zeros drift).
5. **Master → learning (compliant) mode** so it can be moved by hand. If the
   master resists motion, it is still in stiff/servo mode — motion data will
   be flat and the exercise is pointless.
6. **Actor → normal (stiff) mode**: learning/compliant mode **OFF**, so its
   trajectory controller accepts `FollowJointTrajectory` goals.
7. Leave both powered on and connected. Nothing needs to be installed
   *on* the robots: their stock ROS 1 stack + rosbridge server is the interface.

## 3. Workstation: this repo (5 minutes)

```bash
cd ~/projects/niryo_teleop
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate          # uv venv, --system-site-packages (see README)
colcon build
source install/setup.bash
```

## 4. Workstation: the official driver bridge (one-time)

This repo deliberately does **not** talk TCP to the robot. The officially
supported path is `ned-ros2-driver`, which bridges the robot's ROS 1 stack to
ROS 2 over rosbridge (`:9090`). It lives in a **separate** workspace at
`~/projects/niryo_driver_ws` (not `~/ros2_drivers_ws`):

```bash
mkdir -p ~/projects/niryo_driver_ws/src
cd ~/projects/niryo_driver_ws/src
git clone https://github.com/NiryoRobotics/ned-ros2-driver.git
cd ~/projects/niryo_driver_ws
# driver venv uses --system-site-packages so /opt/ros/jazzy stays visible:
uv venv --system-site-packages --python /usr/bin/python3.12 --seed
source .venv/bin/activate
uv pip install colcon-common-extensions colcon-ros "pytest==7.4.4" roslibpy
# IMPORTANT (Jazzy/Ubuntu 24.04): system zope.interface is 6.1, which lacks
# InterfaceClass.__or__. autobahn>=24 uses `ITransport | IProcessTransport`
# and crashes at import with `TypeError: unsupported operand ... InterfaceClass`.
# Pin the last Union-based release in the driver venv:
.venv/bin/python -m pip install "autobahn==23.6.2"
# Runtime message deps required by package.xml exec_depends (else
# `Failed to create driver: No module named 'control_msgs'` after the
# autobahn fix):
sudo apt install -y ros-jazzy-control-msgs ros-jazzy-controller-manager-msgs ros-jazzy-moveit-msgs
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/jazzy/setup.bash
colcon build
```

This project keeps a dedicated config for the master arm alongside the
upstream default, at
`src/ned-ros2-driver/niryo_ned_ros2_driver/config/master_driver.yaml`
(upstream `config/drivers_list.yaml` untouched):

```yaml
rosbridge_port: 9090
robot_namespaces:
  - "master_raw"
robot_ips:
  - "169.254.200.200"   # <-- master Ned2, verified pingable
```

Rebuild after adding/editing that file so it is reinstalled to
`install/.../share/niryo_ned_ros2_driver/config/master_driver.yaml`
(`setup.py` globs `config/*.yaml`):

```bash
cd ~/projects/niryo_driver_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --packages-select niryo_ned_ros2_driver
```

Launch and verify the raw driver topics exist (activate the driver venv
first so `python3` resolves to `.venv/bin/python` with the pinned autobahn):

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/niryo_driver_ws/.venv/bin/activate
source ~/projects/niryo_driver_ws/install/setup.bash
ros2 launch niryo_ned_ros2_driver driver.launch.py drivers_list_file:=$HOME/projects/niryo_driver_ws/src/ned-ros2-driver/niryo_ned_ros2_driver/config/master_driver.yaml
# new terminal:
ros2 topic list | grep joint_states     # expect /master_raw/joint_states
ros2 topic hz --window 30 /master_raw/joint_states   # note the real rate
```

For two robots, use the dual config example in this repo
(`src/niryo_teleop/config/dual_robot_drivers_list.yaml`, same required
`robot_ips`/`robot_namespaces` format, one shared `rosbridge_port`):

```yaml
rosbridge_port: 9090
robot_namespaces: ["master_raw", "actor_raw"]
robot_ips: ["<MASTER_IP>", "<ACTOR_IP>"]  # fill in both, then pass its path
```

```bash
ros2 launch niryo_ned_ros2_driver driver.launch.py drivers_list_file:=<dual_robot_config>
ros2 topic hz /master_raw/joint_states
ros2 topic hz /actor_raw/joint_states
ros2 action list | grep follow_joint_trajectory
# expect /actor_raw/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory
```

Keep only `joint_states` (plus `tf` if you need it) in the driver's topic
whitelist to keep the link lean.

## 5. Point this repo at the driver

Edit `src/niryo_teleop/config/master_params.yaml`:

- `input_topic`: the driver's topic from step 4 (e.g. `/master_raw/joint_states`;
  `/joint_states` if you ran the driver without a namespace).
- The relay republishes every valid message immediately with a workstation-local
  timestamp (no rate limiting, no `publish_rate_hz` to tune).
- Leave `joint_names` as `[joint_1..joint_6]` and `require_exact_joint_names`
  on unless you know why to relax it.

Rebuild so the config is reinstalled:

```bash
cd ~/projects/niryo_teleop
colcon build && source install/setup.bash
```

## 6. Run the teleoperation pipeline

Terminal 1 — driver with the dual config (from step 4). Terminal 2 — this project:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch niryo_teleop full_teleop.launch.py
```

You should see all four nodes start:

```text
master_joint_state_publisher: /master_raw/joint_states -> /master/joint_states ...
teleoperation_controller: /master/joint_states -> /actor/joint_cmd @ 50.0 Hz ...
actor_controller: /actor/joint_cmd -> /actor_raw/.../follow_joint_trajectory @ 10.0 Hz (INITIALIZING...)
actor_joint_state_publisher: /actor_raw/joint_states -> /actor/joint_states ...
```

Startup sequence: the actor controller waits for the action server, a fresh
command, and fresh measured actor state, then moves the actor once to the
master's current pose (`initial_sync_duration_sec`, default 3 s) and enters
`LIVE_TELEOP`. **Keep the master stationary and the actor workspace clear
during this move — the actor moves automatically.**

## 7. First two-robot experiment

1. Confirm topics, types, and the action:
   ```bash
   ros2 topic list | grep -E 'master|actor'
   ros2 topic info /master/joint_states   # sensor_msgs/msg/JointState
   ros2 topic info /actor/joint_cmd       # trajectory_msgs/msg/JointTrajectory
   ros2 topic info /actor/joint_states    # sensor_msgs/msg/JointState
   ros2 action list | grep follow_joint_trajectory
   ```
2. Check all streams flow:
   ```bash
   ros2 topic hz /master_raw/joint_states
   ros2 topic hz /actor_raw/joint_states
   ros2 topic hz /master/joint_states
   ros2 topic hz /actor/joint_cmd
   ros2 topic hz /actor/joint_states
   ```
3. Wait for `entering LIVE_TELEOP` in the actor log (sync move done).
4. Grasp the **master only** (learning mode!) and slowly move joint 1 through
   a tiny arc:
   ```bash
   ros2 topic echo --field position /master/joint_states
   ros2 topic echo --field position /actor/joint_states  # should follow
   ```
   If the actor side doesn't move, check the actor log for
   `INITIALIZING` (still syncing), stale warnings, or action rejections.
5. Only once single-joint following is confirmed: slow multi-joint motions.
   Only touch `joint_scale` / `joint_offsets` in `teleop_params.yaml`, or the
   actor sync/streaming values in `actor_params.yaml`, after identity tracking
   works (rebuild + relaunch after edits; params load at startup).

## 8. Safety rules for the physical sessions

- **Hands on the master only, and hold it still during startup.** The actor
  moves automatically for its sync maneuver as soon as `full_teleop` starts;
  treat every launch as if the actor will move immediately. Keep its workspace
  clear of people, cables, and the bench edge.
- **Slow first.** Start with small, slow motions; `time_from_start_sec`
  (default 0.2 s) is the command horizon — shorter is snappier but harsher.
- **Respect the stale behavior:** if the master/driver drops out, the
  controller goes silent by design (`waiting for master state…`) and the actor
  stops receiving new goals. Don't "fix" silence by replaying old commands.
- **Never leave a live pipeline unattended** with powered arms in reach of
  people, cables, or the bench edge.
- **Stop order:** Ctrl-C the teleop launch first, then the driver; power down
  arms when done.

## 9. Troubleshooting

| Symptom | Likely cause → fix |
|---|---|
| `ros2_driver` dies: `TypeError ... InterfaceClass` in `autobahn/twisted/util.py: peer2str` | autobahn>=24 needs `zope.interface>=7` but venv resolves system 6.1 via `--system-site-packages` → in driver venv run `.venv/bin/python -m pip install "autobahn==23.6.2"`, rebuild, relaunch with venv activated |
| `Failed to create driver: No module named 'control_msgs'` | Missing runtime msg deps → `sudo apt install -y ros-jazzy-control-msgs ros-jazzy-controller-manager-msgs ros-jazzy-moveit-msgs`, then relaunch |
| `Failed to create driver: Timeout to connect to ROSBridge at ws://...:9090` | Robot off / cable unplugged / wrong IP / rosbridge down → `ping -c 3 169.254.200.200`, check TCP 9090, power + Ethernet, retry launch (takes ~12 s to init when healthy) |
| No `/master_raw/joint_states` or no `/actor_raw/joint_states` | Driver misconfigured / robot unreachable → re-check IPs, `ping` both, rosbridge port 9090, driver logs |
| `no joint_states received yet on …` | `input_topic` doesn't match the driver's (namespaced?) topic → fix `master_params.yaml`, rebuild |
| `dropping invalid master state (unexpected joint names…)` | Wrong robot model or namespace collision → verify 6 joints `joint_1..6` via `ros2 topic echo --once` |
| `/master/joint_states` flat while moving arm | Master not in learning/compliant mode → enable in NiryoStudio |
| `/actor/joint_cmd` silent but master flows | Stamp too old (`maximum_state_age_sec`) or RX gap (`stale_timeout_sec`) → check clocks, raise slightly, or reduce rates |
| Jittery rates over Wi-Fi | Link jitter → switch to Ethernet, whitelist only needed topics |
| Node aborts: `param joint_scale has length…` | Vector param length ≠ 6 → fix `teleop_params.yaml` |
| Actor stuck in `INITIALIZING` | Action server/cmd/measured state missing → `ros2 action list`, check `/actor/joint_cmd` flows and actor in stiff (non-learning) mode; see actor log reason |
| Live goals finish with status 6 / driver `EXECUTING→CANCELED` traceback | Overlapping goals preempted on the robot; the driver bridge cannot handle preemption → update `actor_controller` (serialized goals), never stream faster than completions |
| Goals ABORTED (status 6) with no driver traceback, single-goal CLI succeeds | Goal content rejected by the Niryo controller: horizons below `min_goal_duration_sec` (default 1.0 s), a first point at `t=0`, or a workstation-stamped header (epoch-0 means start-now) → keep the floor, use single-point sync moves, zero the goal stamp; verify with `ros2 topic echo --once /actor/joint_cmd` |
| Actor goal rejected/aborted | Controller limits or server state → check limits vs target, action server logs; actor stays gated and retries toward newest target |

Site-specific values you must fill in on the day: robot hostname/IP and driver
namespace (hence `input_topic`). Everything else ships with working defaults.
