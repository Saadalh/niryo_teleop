"""Collection config loading and validation (no hardware, no ROS)."""
from pathlib import Path

import pytest

pytest.importorskip('lerobot')

from lerobot_robot_niryo.config import (  # noqa: E402
    load_settings,
    resolve_datasets_dir,
)


def test_shipped_config_loads():
    settings = load_settings()
    assert settings.fps > 0
    assert settings.camera.name
    assert len(settings.joints.names) == len(settings.joints.limits_min)
    assert len(settings.joints.names) == len(settings.joints.limits_max)
    assert all(
        low <= high
        for low, high in zip(settings.joints.limits_min, settings.joints.limits_max)
    )
    assert settings.robots.master_ip and settings.robots.actor_ip
    assert settings.max_episode_seconds > 0


def test_relative_datasets_dir_resolves_against_repo_root():
    settings = load_settings()
    settings.datasets_dir = 'datasets'
    resolved = resolve_datasets_dir(settings, repo_root=Path('/tmp/repo'))
    assert resolved == Path('/tmp/repo/datasets').resolve()


def test_absolute_datasets_dir_is_kept(tmp_path):
    settings = load_settings()
    settings.datasets_dir = str(tmp_path / 'sets')
    assert resolve_datasets_dir(settings) == tmp_path / 'sets'


def _write_config(path: Path, text: str) -> Path:
    path.write_text(text)
    return path


def _base_config(joints: str) -> str:
    return (
        'collection:\n'
        '  fps: 30\n'
        '  camera:\n'
        '    name: "front"\n'
        '  joints:\n'
        f'{joints}'
        '  teleop:\n'
        '    control_rate_hz: 10.0\n'
    )


def test_mismatched_joint_limits_are_rejected(tmp_path):
    config = _write_config(
        tmp_path / 'bad.yaml',
        _base_config(
            '    names: ["joint_1", "joint_2"]\n'
            '    limits_min: [-1.0]\n'
            '    limits_max: [1.0, 1.0]\n'
        ),
    )
    with pytest.raises(ValueError, match='limits'):
        load_settings(config)


def test_inverted_joint_limits_are_rejected(tmp_path):
    config = _write_config(
        tmp_path / 'bad.yaml',
        _base_config(
            '    names: ["joint_1", "joint_2"]\n'
            '    limits_min: [2.0, -1.0]\n'
            '    limits_max: [1.0, 1.0]\n'
        ),
    )
    with pytest.raises(ValueError, match='limits'):
        load_settings(config)


def test_invalid_camera_name_is_rejected(tmp_path):
    config = _write_config(
        tmp_path / 'bad.yaml',
        """
collection:
  camera:
    name: "front camera"
  joints:
    names: ["joint_1"]
    limits_min: [0.0]
    limits_max: [1.0]
""",
    )
    with pytest.raises(ValueError, match='camera.name'):
        load_settings(config)


def test_non_positive_fps_is_rejected(tmp_path):
    config = _write_config(
        tmp_path / 'bad.yaml',
        """
collection:
  fps: 0
  joints:
    names: ["joint_1"]
    limits_min: [0.0]
    limits_max: [1.0]
""",
    )
    with pytest.raises(ValueError, match='fps'):
        load_settings(config)
