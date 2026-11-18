"""Collection configuration loading and validation."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, List

import yaml


@dataclass
class CameraSettings:
    enabled: bool = True
    # Dataset feature name: observation.images.<name>
    name: str = 'front'
    index_or_path: int | str = '/dev/video0'
    width: int = 640
    height: int = 480
    fps: int = 30
    display_mode: str = 'rerun'


@dataclass
class RobotSettings:
    master_ip: str = ''
    actor_ip: str = ''
    calibrate: bool = True
    master_learning_mode: bool = True
    actor_learning_mode: bool = False


@dataclass
class TopicSettings:
    master: str = '/master/joint_states'
    actor_state: str = '/actor/joint_states'
    actor_cmd: str = '/actor/joint_cmd'
    actor_status: str = '/actor/controller_status'
    enable_service: str = '/actor_controller/enable'


@dataclass
class JointSettings:
    names: List[str] = field(
        default_factory=lambda: [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
        ]
    )
    limits_min: List[float] = field(default_factory=list)
    limits_max: List[float] = field(default_factory=list)


@dataclass
class TeleopSettings:
    control_rate_hz: float = 10.0
    time_from_start_sec: float = 0.2


@dataclass
class CollectionSettings:
    datasets_dir: str = 'datasets'
    default_name_prefix: str = 'niryo-dataset'
    task: str = 'Teleoperate the Ned2'
    fps: int = 30
    max_episode_seconds: float = 300.0
    max_episodes: int = 0
    # How long to wait for the state topics and for the actor to reach
    # LIVE_TELEOP after enabling the controller.
    stream_timeout_sec: float = 20.0
    enable_timeout_sec: float = 30.0
    camera: CameraSettings = field(default_factory=CameraSettings)
    robots: RobotSettings = field(default_factory=RobotSettings)
    topics: TopicSettings = field(default_factory=TopicSettings)
    joints: JointSettings = field(default_factory=JointSettings)
    teleop: TeleopSettings = field(default_factory=TeleopSettings)


def find_repo_root() -> Path:
    """Locate the repository root from this file, falling back to the CWD."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'src' / 'niryo_teleop').is_dir():
            return parent
    return Path.cwd()


def default_config_path() -> Path:
    env_path = os.environ.get('NIRYO_COLLECTION_CONFIG')
    if env_path:
        return Path(env_path).expanduser()
    candidate = (
        find_repo_root()
        / 'src'
        / 'niryo_teleop'
        / 'config'
        / 'collection_params.yaml'
    )
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(
        'could not locate collection_params.yaml; pass --config or set '
        'NIRYO_COLLECTION_CONFIG'
    )


def _mapping(value: Any, name: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be a mapping')
    return value


def _camera(raw: dict) -> CameraSettings:
    index_or_path = raw.get('index_or_path', '/dev/video0')
    if isinstance(index_or_path, str) and index_or_path.isdigit():
        index_or_path = int(index_or_path)
    return CameraSettings(
        enabled=bool(raw.get('enabled', True)),
        name=str(raw.get('name', 'front')),
        index_or_path=index_or_path,
        width=int(raw.get('width', 640)),
        height=int(raw.get('height', 480)),
        fps=int(raw.get('fps', 30)),
        display_mode=str(raw.get('display_mode', 'rerun')),
    )


def load_settings(path: str | Path | None = None) -> CollectionSettings:
    """Load and validate the collection YAML."""
    config_path = Path(path).expanduser() if path else default_config_path()
    data = yaml.safe_load(config_path.read_text())
    root = _mapping(data, 'config')
    collection = _mapping(root.get('collection'), 'collection')

    camera = _camera(_mapping(collection.get('camera', {}), 'camera'))
    if camera.width <= 0 or camera.height <= 0 or camera.fps <= 0:
        raise ValueError('camera width, height, and fps must be positive')
    if not camera.name or not all(
        char.isalnum() or char in '_-' for char in camera.name
    ):
        raise ValueError(
            'camera.name must be a non-empty identifier (letters, digits, _ or -)'
        )

    robots_raw = _mapping(collection.get('robots', {}), 'robots')
    robots = RobotSettings(
        master_ip=str(robots_raw.get('master_ip', '')),
        actor_ip=str(robots_raw.get('actor_ip', '')),
        calibrate=bool(robots_raw.get('calibrate', True)),
        master_learning_mode=bool(robots_raw.get('master_learning_mode', True)),
        actor_learning_mode=bool(robots_raw.get('actor_learning_mode', False)),
    )

    topics_raw = _mapping(collection.get('topics', {}), 'topics')
    topics = TopicSettings(
        master=str(topics_raw.get('master', '/master/joint_states')),
        actor_state=str(topics_raw.get('actor_state', '/actor/joint_states')),
        actor_cmd=str(topics_raw.get('actor_cmd', '/actor/joint_cmd')),
        actor_status=str(
            topics_raw.get('actor_status', '/actor/controller_status')
        ),
        enable_service=str(
            topics_raw.get('enable_service', '/actor_controller/enable')
        ),
    )

    joints_raw = _mapping(collection.get('joints', {}), 'joints')
    names = [str(name) for name in joints_raw.get('names', [])]
    limits_min = [float(value) for value in joints_raw.get('limits_min', [])]
    limits_max = [float(value) for value in joints_raw.get('limits_max', [])]
    if not names:
        raise ValueError('joints.names must not be empty')
    if len(limits_min) != len(names) or len(limits_max) != len(names):
        raise ValueError('joints.limits_min/limits_max must match joints.names')
    if any(low > high for low, high in zip(limits_min, limits_max)):
        raise ValueError('joints.limits_min must be <= joints.limits_max')
    joints = JointSettings(
        names=names, limits_min=limits_min, limits_max=limits_max
    )

    teleop_raw = _mapping(collection.get('teleop', {}), 'teleop')
    teleop = TeleopSettings(
        control_rate_hz=float(teleop_raw.get('control_rate_hz', 10.0)),
        time_from_start_sec=float(teleop_raw.get('time_from_start_sec', 0.2)),
    )
    if teleop.control_rate_hz <= 0:
        raise ValueError('teleop.control_rate_hz must be positive')

    fps = int(collection.get('fps', camera.fps))
    if fps <= 0:
        raise ValueError('collection.fps must be positive')

    stream_timeout_sec = float(collection.get('stream_timeout_sec', 20.0))
    if stream_timeout_sec <= 0:
        raise ValueError('collection.stream_timeout_sec must be positive')
    enable_timeout_sec = float(collection.get('enable_timeout_sec', 30.0))
    if enable_timeout_sec <= 0:
        raise ValueError('collection.enable_timeout_sec must be positive')

    settings = CollectionSettings(
        datasets_dir=str(collection.get('datasets_dir', 'datasets')),
        default_name_prefix=str(
            collection.get('default_name_prefix', 'niryo-dataset')
        ),
        task=str(collection.get('task', 'Teleoperate the Ned2')),
        fps=fps,
        max_episode_seconds=float(
            collection.get('max_episode_seconds', 300.0)
        ),
        max_episodes=int(collection.get('max_episodes', 0)),
        stream_timeout_sec=stream_timeout_sec,
        enable_timeout_sec=enable_timeout_sec,
        camera=camera,
        robots=robots,
        topics=topics,
        joints=joints,
        teleop=teleop,
    )
    if settings.max_episode_seconds <= 0:
        raise ValueError('collection.max_episode_seconds must be positive')
    return settings


def resolve_datasets_dir(
    settings: CollectionSettings, repo_root: Path | None = None
) -> Path:
    """Resolve ``datasets_dir`` relative to the repository root."""
    path = Path(settings.datasets_dir).expanduser()
    if path.is_absolute():
        return path
    root = repo_root or find_repo_root()
    return (root / path).resolve()
