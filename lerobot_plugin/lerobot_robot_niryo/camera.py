"""Camera construction and live preview for collection."""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)


def make_camera(settings):
    """Build and connect a LeRobot OpenCV camera."""
    from lerobot.cameras.opencv import OpenCVCamera
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

    config = OpenCVCameraConfig(
        index_or_path=settings.index_or_path,
        fps=settings.fps,
        width=settings.width,
        height=settings.height,
    )
    camera = OpenCVCamera(config)
    camera.connect()
    return camera


class CameraPreview:
    """Stream camera frames to LeRobot's Rerun viewer (OpenCV fallback)."""

    def __init__(self, camera, *, display_mode: str = 'rerun', fps: float = 10.0) -> None:
        self.camera = camera
        self.display_mode = display_mode
        self.fps = max(1.0, float(fps))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._backend = 'none'
        self._window_name = 'Niryo camera (press q to close preview)'

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._backend = self._select_backend()
        if self._backend == 'none':
            return
        self._thread = threading.Thread(
            target=self._loop, name='camera_preview', daemon=True
        )
        self._thread.start()

    def _select_backend(self) -> str:
        if self.display_mode != 'rerun':
            return self._select_opencv()
        try:
            from lerobot.utils.visualization_utils import init_visualization

            init_visualization('rerun', session_name='niryo_collection')
            return 'rerun'
        except Exception as exc:
            logger.warning(
                'Rerun preview unavailable (%s); falling back to OpenCV', exc
            )
            return self._select_opencv()

    def _select_opencv(self) -> str:
        try:
            import cv2  # noqa: F401

            return 'opencv'
        except Exception as exc:
            logger.warning('OpenCV preview unavailable (%s)', exc)
            return 'none'

    def _loop(self) -> None:
        if self._backend == 'opencv':
            import cv2

            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        while not self._stop.is_set():
            try:
                frame = self.camera.read_latest(max_age_ms=1000)
            except Exception:
                time.sleep(0.05)
                continue
            if self._backend == 'rerun':
                from lerobot.utils.visualization_utils import (
                    log_visualization_data,
                )

                log_visualization_data(
                    'rerun', observation={'camera': frame}
                )
            elif self._backend == 'opencv':
                import cv2

                cv2.imshow(
                    self._window_name,
                    cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                )
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._stop.set()
            time.sleep(1.0 / self.fps)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._backend == 'opencv':
            try:
                import cv2

                cv2.destroyWindow(self._window_name)
            except Exception:
                pass

    def shutdown(self) -> None:
        self.stop()
        if self._backend == 'rerun':
            try:
                from lerobot.utils.visualization_utils import (
                    shutdown_visualization,
                )

                shutdown_visualization('rerun')
            except Exception:
                pass
