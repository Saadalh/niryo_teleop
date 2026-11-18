"""Prompt-driven LeRobot dataset collection for the Niryo Ned2 teleop stack.

Session flow (terminal prompts only, no extra input devices):

1. Confirm the bench is safe.
2. pyniryo auto-calibrates both arms and sets their learning modes.
3. Confirm the environment is ready.
4. The ROS runtime starts with the actor controller gated, and the camera
   preview opens in its own window.
5. Between episodes the actor follows the master. Enter starts an episode,
   Enter stops and saves it.
6. Ctrl-X between episodes ends the session; the dataset is finalized and
   named.

The collector never implements a dataset format: episodes are written by
LeRobot's ``record_loop`` into a ``LeRobotDataset`` and saved with
``save_episode``.
"""
from __future__ import annotations

import argparse
import logging
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Sequence

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.cycle_timer import CycleTimer
from lerobot.utils.feature_utils import combine_feature_dicts

from .calibration import calibrate_robots
from .camera import CameraPreview
from .config import (
    CollectionSettings,
    load_settings,
    resolve_datasets_dir,
)
from .config_niryo_ned2 import (
    NiryoNed2FollowerConfig,
    NiryoNed2LeaderConfig,
)
from .naming import (
    default_dataset_name,
    unique_dataset_name,
    validate_dataset_name,
)
from .ros_io import ensure_rclpy, shutdown_rclpy
from .terminal import TerminalIO

LOGGER = logging.getLogger('niryo_collect')

LAUNCH_FILE = 'collection.launch.py'


# --------------------------------------------------------------------------
# Hardware configuration
# --------------------------------------------------------------------------


def make_follower_config(settings: CollectionSettings) -> NiryoNed2FollowerConfig:
    """Build the actor adapter config, including the optional camera."""
    cameras: Dict[str, object] = {}
    if settings.camera.enabled:
        cameras[settings.camera.name] = OpenCVCameraConfig(
            index_or_path=settings.camera.index_or_path,
            fps=settings.camera.fps,
            width=settings.camera.width,
            height=settings.camera.height,
        )
    return NiryoNed2FollowerConfig(
        master_topic=settings.topics.master,
        actor_state_topic=settings.topics.actor_state,
        actor_cmd_topic=settings.topics.actor_cmd,
        joint_names=list(settings.joints.names),
        joint_limits_min=list(settings.joints.limits_min),
        joint_limits_max=list(settings.joints.limits_max),
        time_from_start_sec=settings.teleop.time_from_start_sec,
        cameras=cameras,
    )


def make_leader_config(settings: CollectionSettings) -> NiryoNed2LeaderConfig:
    """Build the master adapter config."""
    return NiryoNed2LeaderConfig(
        master_topic=settings.topics.master,
        actor_state_topic=settings.topics.actor_state,
        actor_cmd_topic=settings.topics.actor_cmd,
        joint_names=list(settings.joints.names),
        joint_limits_min=list(settings.joints.limits_min),
        joint_limits_max=list(settings.joints.limits_max),
        time_from_start_sec=settings.teleop.time_from_start_sec,
    )


def build_dataset_features(robot, teleop, processors) -> dict:
    """Dataset feature schema: actor state observations + leader actions."""
    teleop_action_processor, robot_action_processor, observation_processor = (
        processors
    )
    del robot_action_processor  # identity processor, no feature contribution
    return combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(
                action=robot.action_features
            ),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=observation_processor,
            initial_features=create_initial_features(
                observation=robot.observation_features
            ),
            use_videos=True,
        ),
    )


# --------------------------------------------------------------------------
# Runtime + teleoperation helpers
# --------------------------------------------------------------------------


class RuntimeProcess:
    """The ROS relay/controller runtime, launched as a child process."""

    def __init__(self, launch_file: str = LAUNCH_FILE) -> None:
        self.launch_file = launch_file
        self._process: Optional[subprocess.Popen] = None

    def start(self) -> None:
        ros2 = shutil.which('ros2')
        if ros2 is None:
            raise RuntimeError(
                'ros2 was not found on PATH; source the ROS 2 installation '
                'or pass --no-runtime if the nodes are already running'
            )
        self._process = subprocess.Popen(
            [ros2, 'launch', 'niryo_teleop', self.launch_file],
            start_new_session=True,
        )

    def stop(self) -> None:
        process = self._process
        if process is None:
            return
        self._process = None
        if process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5.0)


class TeleopBridge:
    """Master -> actor teleoperation while no episode is being recorded.

    Publishes the leader's clamped action unchanged, so the motion seen
    between episodes matches what the dataset records during one. A missing
    or stale master sample publishes nothing; stale commands are never
    replayed.
    """

    def __init__(
        self,
        leader,
        follower,
        *,
        rate_hz: float,
        report=print,
    ) -> None:
        self.leader = leader
        self.follower = follower
        self.period = 1.0 / float(rate_hz)
        self.report = report
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._warned = False

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self._warned = False
        self._thread = threading.Thread(
            target=self._loop, name='teleop_bridge', daemon=True
        )
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self.follower.send_action(self.leader.get_action())
                self._warned = False
            except RuntimeError as exc:
                if not self._warned:
                    self.report(f'teleoperation paused: {exc}')
                    self._warned = True
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, self.period - elapsed))

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=2.0)


class EpisodeStopWatcher:
    """Turns an Enter keypress into ``events['exit_early']`` for record_loop."""

    def __init__(self, terminal: TerminalIO) -> None:
        self.terminal = terminal
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self, events: dict) -> None:
        if not self.terminal.is_tty():
            return  # non-interactive: rely on the episode time limit
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, args=(events,), name='episode_stop', daemon=True
        )
        self._thread.start()

    def _run(self, events: dict) -> None:
        try:
            if self.terminal.wait_for_enter(stop_event=self._stop) == 'enter':
                events['exit_early'] = True
        except KeyboardInterrupt:
            events['exit_early'] = True

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=2.0)


# --------------------------------------------------------------------------
# Session
# --------------------------------------------------------------------------


class CollectionSession:
    """One interactive collection session."""

    def __init__(
        self,
        settings: CollectionSettings,
        *,
        datasets_dir: Path,
        terminal: TerminalIO,
        start_runtime: bool = True,
        calibrate: bool = True,
        max_episodes: int = 0,
    ) -> None:
        self.settings = settings
        self.datasets_dir = datasets_dir
        self.terminal = terminal
        self.start_runtime = start_runtime
        self.calibrate = calibrate
        self.max_episodes = max_episodes

        self.runtime: Optional[RuntimeProcess] = None
        self.robot = None
        self.teleop = None
        self.bridge: Optional[TeleopBridge] = None
        self.preview: Optional[CameraPreview] = None
        self.dataset: Optional[LeRobotDataset] = None
        self._processors = make_default_processors()
        self._timer: Optional[CycleTimer] = None

    # -- top level --------------------------------------------------------

    def run(self) -> Path:
        self._confirm_safe()
        if self.calibrate:
            self._calibrate()
        self._confirm_ready()
        try:
            self._start_runtime()
            self._connect_adapters()
            self._start_preview()
            # The controller needs a live command stream to sync to, so the
            # bridge runs before enabling it. While gated, commands are
            # ignored, so nothing moves yet.
            if self.bridge is None:
                raise RuntimeError('teleoperation bridge was not created')
            self.bridge.start()
            self._enable_actor()
            name = self._create_dataset()
            with VideoEncodingManager(self.dataset):
                self._episode_loop()
            return self._finish_dataset(name)
        finally:
            self._cleanup()

    # -- prompts ----------------------------------------------------------

    def _confirm_safe(self) -> None:
        print('\n=== Niryo Ned2 dataset collection ===')
        print(
            'Safety check: clear the workspace around both arms, keep a hand '
            'on the emergency stop, and make sure the actor can move freely.'
        )
        self.terminal.wait_for_enter(
            'Press Enter to confirm the bench is safe (Ctrl-C to abort).'
        )

    def _calibrate(self) -> None:
        print('\nCalibrating both robots with pyniryo...')
        calibrate_robots(
            self.settings.robots.master_ip,
            self.settings.robots.actor_ip,
            master_learning_mode=self.settings.robots.master_learning_mode,
            actor_learning_mode=self.settings.robots.actor_learning_mode,
        )
        print('Calibration finished.')

    def _confirm_ready(self) -> None:
        print(
            '\nEnvironment check: both robots are powered and connected, the '
            'camera is plugged in, and the workspace is still clear.'
        )
        self.terminal.wait_for_enter(
            'Press Enter when the environment is ready.'
        )

    # -- setup ------------------------------------------------------------

    def _start_runtime(self) -> None:
        if not self.start_runtime:
            print('\nAssuming the ROS runtime and driver are already running.')
            return
        print('\nStarting the ROS runtime (relays + gated actor controller)...')
        self.runtime = RuntimeProcess()
        self.runtime.start()

    def _connect_adapters(self) -> None:
        ensure_rclpy()
        self.teleop = make_teleoperator_from_config(make_leader_config(self.settings))
        self.robot = make_robot_from_config(make_follower_config(self.settings))
        # Teleoperator first so the actor is not left idle while the master
        # bridge initialises (same order as lerobot-record).
        self.teleop.connect()
        self.robot.connect()

        timeout = self.settings.stream_timeout_sec
        print(f'\nWaiting for joint states (up to {timeout:.0f} s)...')
        if not self.teleop.wait_for_master_state(timeout):
            raise RuntimeError(
                f'no master joint states on {self.settings.topics.master}'
            )
        if not self.robot.wait_for_actor_state(timeout):
            raise RuntimeError(
                f'no actor joint states on {self.settings.topics.actor_state}'
            )
        print('Joint states received.')
        self.bridge = TeleopBridge(
            self.teleop,
            self.robot,
            rate_hz=self.settings.teleop.control_rate_hz,
        )

    def _start_preview(self) -> None:
        if not self.settings.camera.enabled:
            print('\nCamera disabled in the config; recording without video.')
            return
        camera = next(iter(self.robot.cameras.values()))
        self.preview = CameraPreview(
            camera,
            display_mode=self.settings.camera.display_mode,
            fps=self.settings.camera.fps,
        )
        self.preview.start()
        print(
            'Camera preview open in a separate window '
            f'(observation.images.{self.settings.camera.name}).'
        )

    def _enable_actor(self) -> None:
        if self.bridge is None:
            raise RuntimeError('teleoperation bridge was not created')
        print('\nEnabling the actor controller (the actor syncs to the master)...')
        success, message = self.robot.enable_controller()
        if not success:
            raise RuntimeError(f'actor controller refused to enable: {message}')

        deadline = time.monotonic() + self.settings.enable_timeout_sec
        while time.monotonic() < deadline:
            status = self.robot.latest_status()
            if status == 'LIVE_TELEOP':
                print('Actor is live; teleoperation is active.')
                return
            time.sleep(0.1)
        status = self.robot.latest_status()
        raise RuntimeError(
            'actor did not reach LIVE_TELEOP '
            f'(last status: {status}); check the driver and the action server'
        )

    # -- dataset ----------------------------------------------------------

    def _create_dataset(self) -> str:
        self.datasets_dir.mkdir(parents=True, exist_ok=True)
        name = unique_dataset_name(
            self.datasets_dir,
            default_dataset_name(self.settings.default_name_prefix),
        )
        features = build_dataset_features(
            self.robot, self.teleop, self._processors
        )
        root = self.datasets_dir / name
        print(f'\nRecording into {root}')
        self.dataset = LeRobotDataset.create(
            repo_id=f'local/{name}',
            fps=self.settings.fps,
            root=root,
            robot_type=self.robot.name,
            features=features,
            use_videos=True,
            image_writer_processes=0,
            image_writer_threads=4 * len(self.robot.cameras),
        )
        self._timer = CycleTimer(self.settings.fps)
        return name

    # -- episode loop -----------------------------------------------------

    def _episode_loop(self) -> None:
        if self.bridge is None:
            raise RuntimeError('teleoperation bridge was not created')
        recorded = 0
        while True:
            if self.max_episodes and recorded >= self.max_episodes:
                print(f'\nReached the episode limit ({self.max_episodes}).')
                return
            self._check_actor_status()
            self.bridge.start()
            try:
                choice = self.terminal.wait_for_enter(
                    f'\nMove the master to position the actor. '
                    f'Press Enter to record episode {recorded + 1} '
                    f'(Ctrl-X to finish).',
                    allow_ctrlx=True,
                )
            finally:
                self.bridge.stop()
            if choice != 'enter':
                print(f'\nFinishing after {recorded} episode(s).')
                return
            self._record_episode(recorded + 1)
            recorded += 1

    def _check_actor_status(self) -> None:
        status = self.robot.latest_status()
        if status is not None and status != 'LIVE_TELEOP':
            raise RuntimeError(
                f'actor controller is not live (status: {status}); '
                'refusing to record while the actor cannot follow'
            )

    def _record_episode(self, index: int) -> None:
        print(
            f'\nRecording episode {index} at {self.settings.fps} Hz. '
            'Press Enter to stop.'
        )
        events = {'exit_early': False}
        watcher = EpisodeStopWatcher(self.terminal)
        watcher.start(events)
        started = time.monotonic()
        try:
            record_loop(
                robot=self.robot,
                events=events,
                fps=self.settings.fps,
                teleop_action_processor=self._processors[0],
                robot_action_processor=self._processors[1],
                robot_observation_processor=self._processors[2],
                teleop=self.teleop,
                dataset=self.dataset,
                control_time_s=self.settings.max_episode_seconds,
                single_task=self.settings.task,
                display_data=False,
                timer=self._timer,
            )
        except BaseException:
            self.dataset.clear_episode_buffer()
            raise
        finally:
            watcher.stop()

        duration = time.monotonic() - started
        if not self.dataset.has_pending_frames():
            print('No frames were recorded; episode discarded.')
            self.dataset.clear_episode_buffer()
            return
        print(f'Saving episode {index} ({duration:.1f} s)...')
        self.dataset.save_episode()
        self._timer.log_episode_summary(f'episode {index}')
        self._timer.restart()
        print(f'Episode {index} saved.')

    # -- finish -----------------------------------------------------------

    def _finish_dataset(self, name: str) -> Path:
        if self.dataset is None:
            raise RuntimeError('no dataset was created')
        # VideoEncodingManager already finalized the dataset on exit.
        requested = name
        while True:
            try:
                requested = validate_dataset_name(
                    self.terminal.prompt_text('Dataset name', name)
                )
            except ValueError as exc:
                print(f'Invalid name: {exc}')
                continue
            break
        final_name = unique_dataset_name(self.datasets_dir, requested)
        current = Path(self.dataset.root)
        target = self.datasets_dir / final_name
        if target != current:
            current.rename(target)
        episodes = self.dataset.num_episodes
        frames = self.dataset.num_frames
        print(
            f'\nDataset ready: {target}\n'
            f'  episodes: {episodes}, frames: {frames}, fps: {self.settings.fps}'
        )
        return target

    # -- cleanup ----------------------------------------------------------

    def _cleanup(self) -> None:
        if self.bridge is not None:
            self.bridge.stop()
        if self.preview is not None:
            self.preview.shutdown()
            self.preview = None
        if self.dataset is not None:
            try:
                self.dataset.finalize()
            except Exception:  # already finalized by VideoEncodingManager
                LOGGER.debug('dataset already finalized', exc_info=True)
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                LOGGER.debug('follower disconnect failed', exc_info=True)
            self.robot = None
        if self.teleop is not None:
            try:
                self.teleop.disconnect()
            except Exception:
                LOGGER.debug('leader disconnect failed', exc_info=True)
            self.teleop = None
        if self.runtime is not None:
            self.runtime.stop()
            self.runtime = None
        shutdown_rclpy()


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog='niryo-collect',
        description='Record a LeRobot dataset from Ned2 master/actor teleoperation.',
    )
    parser.add_argument(
        '--config',
        help='collection YAML (default: src/niryo_teleop/config/collection_params.yaml, '
        'or NIRYO_COLLECTION_CONFIG)',
    )
    parser.add_argument(
        '--datasets-dir',
        help='override the dataset root directory (relative paths resolve '
        'against the repository root)',
    )
    parser.add_argument('--task', help='override the task string stored per frame')
    parser.add_argument(
        '--max-episodes',
        type=int,
        default=0,
        help='stop after N episodes (0 = until Ctrl-X)',
    )
    parser.add_argument(
        '--skip-calibration',
        action='store_true',
        help='skip pyniryo calibration (already calibrated robots)',
    )
    parser.add_argument(
        '--no-runtime',
        action='store_true',
        help='do not launch the ROS runtime; assume it is already running',
    )
    parser.add_argument(
        '--no-camera',
        action='store_true',
        help='record without a camera (joint observation/action only)',
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    logging.basicConfig(
        level=logging.INFO, format='%(levelname)s %(name)s: %(message)s'
    )
    args = parse_args(argv)

    settings = load_settings(args.config)
    if args.datasets_dir:
        settings.datasets_dir = args.datasets_dir
    if args.task:
        settings.task = args.task
    if args.max_episodes:
        settings.max_episodes = args.max_episodes
    if args.no_camera:
        settings.camera.enabled = False

    datasets_dir = resolve_datasets_dir(settings)
    session = CollectionSession(
        settings,
        datasets_dir=datasets_dir,
        terminal=TerminalIO(),
        start_runtime=not args.no_runtime,
        calibrate=settings.robots.calibrate and not args.skip_calibration,
        max_episodes=settings.max_episodes,
    )

    try:
        session.run()
    except KeyboardInterrupt:
        print('\nInterrupted; saved episodes are kept.')
        return 130
    except Exception as exc:
        LOGGER.error('collection aborted: %s', exc)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
