"""Interactive terminal prompts (Enter / Ctrl-X) for collection."""
from __future__ import annotations

import os
import select
import sys
import termios
import tty
from contextlib import contextmanager
from typing import Optional, TextIO


class TerminalIO:
    """Blocking prompts with a terminal-friendly Ctrl-X escape."""

    ENTER_KEYS = (b'\r', b'\n')
    CTRL_X = b'\x18'
    CTRL_C = b'\x03'

    def __init__(self, stream: Optional[TextIO] = None) -> None:
        self.stream = stream or sys.stdin

    def is_tty(self) -> bool:
        return bool(self.stream.isatty())

    @contextmanager
    def cbreak(self):
        if not self.is_tty():
            yield
            return
        fd = self.stream.fileno()
        previous = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            yield
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, previous)

    def read_key(self, timeout: Optional[float] = None) -> Optional[bytes]:
        if not self.is_tty():
            char = self.stream.read(1)
            return char.encode() if char else None
        ready, _, _ = select.select([self.stream], [], [], timeout)
        if not ready:
            return None
        return os.read(self.stream.fileno(), 1)

    def wait_for_enter(
        self,
        prompt: str = '',
        *,
        allow_ctrlx: bool = False,
        stop_event=None,
    ) -> str:
        """Wait for Enter (returns ``"enter"``) or Ctrl-X (``"ctrlx"``).

        When *stop_event* is set, the wait ends with ``"stopped"``. End of
        input on a non-interactive stream also returns ``"stopped"``.
        """
        if prompt:
            print(prompt, flush=True)
        with self.cbreak():
            while stop_event is None or not stop_event.is_set():
                key = self.read_key(timeout=0.1)
                if key is None:
                    if not self.is_tty():
                        return 'stopped'
                    continue
                if key in self.ENTER_KEYS:
                    return 'enter'
                if allow_ctrlx and key == self.CTRL_X:
                    return 'ctrlx'
                if key == self.CTRL_C:
                    raise KeyboardInterrupt
        return 'stopped'

    def prompt_text(self, prompt: str, default: str) -> str:
        """Read a free-text value in canonical mode; empty input keeps *default*."""
        try:
            value = input(f'{prompt} [{default}]: ').strip()
        except EOFError:
            value = ''
        return value or default
