#!/usr/bin/env bash
# Read-only environment check for Chunk 2 (never installs/modifies anything).
set -u
echo "== niryo_teleop env check =="
echo "PWD: $(pwd)"
command -v uv >/dev/null && uv --version || echo "MISSING: uv"
python3 --version || echo "MISSING: python3"
test -f /opt/ros/jazzy/setup.bash && echo "ROS jazzy setup.bash: present" || echo "MISSING: /opt/ros/jazzy"
test -d .venv && echo ".venv: present" || echo ".venv: absent (create with: uv venv --system-site-packages --python 3.12)"
test -f .venv/bin/activate && grep -q "include-system-site-packages = true" .venv/pyvenv.cfg 2>/dev/null && echo "venv system-site-packages: enabled" || echo "venv system-site-packages: unknown/absent"
command -v colcon >/dev/null && echo "colcon: $(command -v colcon)" || echo "colcon: not on PATH (expected before venv activation)"
echo "ROS_DISTRO=${ROS_DISTRO:-(unset)} ROS_VERSION=${ROS_VERSION:-(unset)}"
