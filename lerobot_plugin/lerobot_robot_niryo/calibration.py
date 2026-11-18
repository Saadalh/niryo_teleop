"""pyniryo calibration for the dataset collection session."""
from __future__ import annotations

import logging
from typing import Callable, Optional

logger = logging.getLogger(__name__)


def _default_robot_factory(ip_address: str):
    from pyniryo import NiryoRobot

    return NiryoRobot(ip_address)


def calibrate_robot(
    label: str,
    ip_address: str,
    *,
    learning_mode: bool,
    dry_run: bool = False,
    robot_factory: Optional[Callable] = None,
) -> None:
    """Run ``calibrate_auto`` on one robot, then set its learning mode."""
    if dry_run:
        logger.info('[dry-run] calibrate %s at %s', label, ip_address)
        return
    factory = robot_factory or _default_robot_factory
    logger.info('calibrating %s at %s', label, ip_address)
    robot = factory(ip_address)
    try:
        robot.calibrate_auto()
        robot.set_learning_mode(bool(learning_mode))
        logger.info(
            'calibration complete for %s (learning_mode=%s)',
            label,
            bool(learning_mode),
        )
    finally:
        try:
            robot.close_connection()
        except Exception:
            logger.exception('failed to close %s connection', label)


def calibrate_robots(
    master_ip: str,
    actor_ip: str,
    *,
    master_learning_mode: bool = True,
    actor_learning_mode: bool = False,
    dry_run: bool = False,
    robot_factory: Optional[Callable] = None,
) -> None:
    """Calibrate master then actor, before any runtime node can move them."""
    calibrate_robot(
        'master',
        master_ip,
        learning_mode=master_learning_mode,
        dry_run=dry_run,
        robot_factory=robot_factory,
    )
    calibrate_robot(
        'actor',
        actor_ip,
        learning_mode=actor_learning_mode,
        dry_run=dry_run,
        robot_factory=robot_factory,
    )
