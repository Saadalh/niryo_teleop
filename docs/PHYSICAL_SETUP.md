# Physical setup & getting started with Ned2 teleoperation

How to go from boxed robots to a working teleoperation pipeline:

```text
Physical Ned2 (master)
  -> ned-ros2-driver  =>  /joint_states            (official bridge, separate workspace)
  -> master_joint_state_publisher  =>  /master/joint_states   (this repo, Chunk 3)
  -> teleoperation_controller      =>  /actor/joint_cmd       (this repo, Chunk 4)
```

The actor arm is **not** connected yet (Chunk 4 scope stops at `/actor/joint_cmd`).
What you validate here is the full *sensing + mapping* side: move the master by
hand and watch correctly structured actor commands stream out.

## 1. What you need

- 2× Niryo **Ned2** arms with power supplies. Only the **master** is used so far;
  keep the actor powered off or parked out of the way.
- Workstation: Ubuntu 24.04 + ROS 2 Jazzy (see `README.md` — this repo builds
  without touching the system install).
- Network: wired Ethernet strongly recommended (Wi-Fi/hotspot adds jitter to the
  rosbridge link). One switch/router, or direct Ethernet to each robot.
- NiryoStudio installed on the workstation (robot updates, calibration,
  learning-mode toggle). Download from Niryo's site if missing.
- Clear bench space: no obstacles in either arm's reach, emergency stop
  reachable (NiryoStudio stop / power switch).

## 2. Prepare the master robot

1. **Power and connect.** Power the master Ned2, connect it to the same network
   as the workstation (Ethernet preferred). Note its address: hostname looks
   like `ned2-aa-bbb-ccc.local`, or take the DHCP IP from your router.
2. **Check reachability** from the workstation:
   ```bash
   ping -c 3 <robot-hostname-or-ip>
   ```
3. **Update + calibrate in NiryoStudio.** Connect NiryoStudio to the master,
   install any pending robot software update, then run the motor calibration.
   Teleoperation on an uncalibrated arm is meaningless (joint zeros drift).
4. **Enable learning (compliant) mode** on the master so it can be moved by
   hand. If the arm resists motion, it is still in stiff/servo mode — motion
   data will be flat and the exercise is pointless.
5. Leave the master powered on and connected. Nothing needs to be installed
   *on* the robot: its stock ROS 1 stack + rosbridge server is the interface.

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

Launch and verify the raw driver topic exists (activate the driver venv
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

Keep only `joint_states` (plus `tf` if you need it) in the driver's topic
whitelist to keep the link lean.

## 5. Point this repo at the driver

Edit `src/niryo_teleop/config/master_params.yaml`:

- `input_topic`: the driver's topic from step 4 (e.g. `/master_raw/joint_states`;
  `/joint_states` if you ran the driver without a namespace).
- `publish_rate_hz`: set **≥** the measured driver rate for a lossless relay
  (e.g. driver at ~30 Hz → keep 50). A cap below the driver rate throttles
  output by design.
- Leave `joint_names` as `[joint_1..joint_6]` and `require_exact_joint_names`
  on unless you know why to relax it.

Rebuild so the config is reinstalled:

```bash
cd ~/projects/niryo_teleop
colcon build && source install/setup.bash
```

## 6. Run the teleoperation pipeline

Terminal 1 — driver (from step 4). Terminal 2 — this project:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch niryo_teleop teleop.launch.py
```

You should see both nodes start without warnings:

```text
master_joint_state_publisher: /master_raw/joint_states -> /master/joint_states @ ...
teleoperation_controller: /master/joint_states -> /actor/joint_cmd @ 50.0 Hz ...
publishing measured state on /master/joint_states ...
publishing actor commands on /actor/joint_cmd ...
```

## 7. Getting started: move the master, watch the commands

1. Confirm the topics and types:
   ```bash
   ros2 topic list | grep -E 'master|actor'
   ros2 topic info /master/joint_states   # sensor_msgs/msg/JointState
   ros2 topic info /actor/joint_cmd       # trajectory_msgs/msg/JointTrajectory
   ```
2. Grasp the master arm (learning mode!) and slowly move joint 1 through a
   small arc. In another terminal:
   ```bash
   ros2 topic echo --field position /master/joint_states
   ```
   Values must track your hand. If they don't move, the arm isn't compliant or
   you're watching the wrong (namespaced) topic.
3. Check the mapped command mirrors it (default mapping is identity):
   ```bash
   ros2 topic echo --once --field points /actor/joint_cmd
   ```
4. Check rates:
   ```bash
   ros2 topic hz --window 40 /master/joint_states
   ros2 topic hz --window 40 /actor/joint_cmd   # expect ~= control_rate_hz
   ```
5. First experiments: slow single-joint motions, then multi-joint. Only touch
   `joint_scale` / `joint_offsets` in `teleop_params.yaml` once identity
   tracking is confirmed (rebuild + relaunch after edits; params load at
   startup).

## 8. Safety rules for the physical sessions

- **Hands on the master only.** The actor isn't driven yet; when it is (later
  chunk), treat every launch as if both arms will move immediately.
- **Slow first.** Start with small, slow motions; `time_from_start_sec`
  (default 0.2 s) is the command horizon — shorter is snappier but harsher.
- **Respect the stale behavior:** if the master/driver drops out, the
  controller goes silent by design (`waiting for master state…`). Don't
  "fix" silence by replaying old commands.
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
| No `/master_raw/joint_states` | Driver misconfigured / robot unreachable → re-check IP, `ping`, rosbridge port 9090, driver logs |
| `no joint_states received yet on …` | `input_topic` doesn't match the driver's (namespaced?) topic → fix `master_params.yaml`, rebuild |
| `dropping invalid master state (unexpected joint names…)` | Wrong robot model or namespace collision → verify 6 joints `joint_1..6` via `ros2 topic echo --once` |
| `/master/joint_states` flat while moving arm | Master not in learning/compliant mode → enable in NiryoStudio |
| `/actor/joint_cmd` silent but master flows | Stamp too old (`maximum_state_age_sec`) or RX gap (`stale_timeout_sec`) → check clocks, raise slightly, or reduce rates |
| Jittery rates over Wi-Fi | Link jitter → switch to Ethernet, whitelist only needed topics |
| Node aborts: `param joint_scale has length…` | Vector param length ≠ 6 → fix `teleop_params.yaml` |

Site-specific values you must fill in on the day: robot hostname/IP, driver
namespace (hence `input_topic`), and the measured driver rate (hence
`publish_rate_hz`). Everything else ships with working defaults.
