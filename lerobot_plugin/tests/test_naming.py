"""Dataset naming helpers (no hardware, no ROS)."""
from datetime import datetime

import pytest

pytest.importorskip('lerobot')

from lerobot_robot_niryo.naming import (  # noqa: E402
    default_dataset_name,
    unique_dataset_name,
    validate_dataset_name,
)


def test_default_name_uses_prefix_and_timestamp():
    name = default_dataset_name(
        'niryo-dataset', now=datetime(2026, 2, 7, 9, 5, 30)
    )
    assert name == 'niryo-dataset-26-02-07_09-05'


def test_default_name_default_prefix():
    name = default_dataset_name(now=datetime(2026, 12, 31, 23, 59))
    assert name.startswith('niryo-dataset-')
    assert ' ' not in name and '/' not in name


@pytest.mark.parametrize('name', ['dataset', 'a', 'niryo_1', 'Run.2-x', '  padded  '])
def test_validate_accepts_safe_names(name):
    cleaned = validate_dataset_name(name)
    assert cleaned == name.strip()


@pytest.mark.parametrize(
    'name', ['', '   ', '../escape', 'a/b', 'a\\b', '.hidden', '-leading']
)
def test_validate_rejects_unsafe_names(name):
    with pytest.raises(ValueError):
        validate_dataset_name(name)


def test_unique_dataset_name_never_overwrites(tmp_path):
    assert unique_dataset_name(tmp_path, 'run') == 'run'
    (tmp_path / 'run').mkdir()
    assert unique_dataset_name(tmp_path, 'run') == 'run-2'
    (tmp_path / 'run-2').mkdir()
    assert unique_dataset_name(tmp_path, 'run') == 'run-3'
