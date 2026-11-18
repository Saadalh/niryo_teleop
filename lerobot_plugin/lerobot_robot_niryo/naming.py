"""Dataset naming helpers for collection."""
from __future__ import annotations

import re
from datetime import datetime

_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')


def default_dataset_name(prefix: str = 'niryo-dataset', now: datetime | None = None) -> str:
    """Default name: ``<prefix>-YY-MM-DD_HH-MM`` (local time)."""
    timestamp = now or datetime.now()
    return f'{prefix}-{timestamp:%y-%m-%d_%H-%M}'


def validate_dataset_name(name: str) -> str:
    """Return a safe dataset directory name or raise ``ValueError``."""
    candidate = name.strip()
    if not candidate:
        raise ValueError('dataset name must not be empty')
    if candidate in ('.', '..') or '/' in candidate or '\\' in candidate:
        raise ValueError('dataset name must not contain path separators')
    if not _NAME_RE.match(candidate):
        raise ValueError(
            'dataset name may only contain letters, digits, dot, dash, and '
            'underscore, and must start with a letter or digit'
        )
    return candidate


def unique_dataset_name(root, name: str) -> str:
    """Return *name* or ``name-2``, ``name-3``… so no existing dataset is overwritten."""
    from pathlib import Path

    root = Path(root)
    if not (root / name).exists():
        return name
    for index in range(2, 1000):
        candidate = f'{name}-{index}'
        if not (root / candidate).exists():
            return candidate
    raise RuntimeError(f'could not find a free dataset name for {name}')
