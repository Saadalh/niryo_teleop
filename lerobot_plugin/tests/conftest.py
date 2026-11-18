"""Make ``lerobot_robot_niryo`` importable when running the plugin suite.

These tests need ``lerobot``, which lives in the ``vlas`` virtualenv rather
than the workspace ``.venv``. They are therefore not part of the root
``testpaths``; run them explicitly:

    cd /home/rahmlab/projects/niryo_teleop
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
        /home/rahmlab/projects/vlas/.venv/bin/python -m pytest lerobot_plugin/tests -q

``PYTEST_DISABLE_PLUGIN_AUTOLOAD`` keeps ROS's pytest plugins out of that
environment (they need packages the vlas venv does not install).
"""
import sys
from pathlib import Path

PLUGIN_ROOT = Path(__file__).resolve().parents[1]
if str(PLUGIN_ROOT) not in sys.path:
    sys.path.insert(0, str(PLUGIN_ROOT))
