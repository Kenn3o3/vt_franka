from __future__ import annotations

import logging
import threading
import traceback
from collections import deque
from typing import Any


class OperatorLogBuffer(logging.Handler):
    def __init__(self, max_records: int = 500) -> None:
        super().__init__(level=logging.INFO)
        self.max_records = max(1, int(max_records))
        self._entries: deque[dict[str, Any]] = deque(maxlen=self.max_records)
        self._lock = threading.Lock()
        self._sequence = 0

    def emit(self, record: logging.LogRecord) -> None:
        message = record.getMessage()
        if record.exc_info:
            message = f"{message}\n{''.join(traceback.format_exception(*record.exc_info)).rstrip()}"
        entry = {
            "seq": self._sequence,
            "created": record.created,
            "level": record.levelname,
            "logger": record.name,
            "message": message,
        }
        with self._lock:
            self._entries.append(entry)
            self._sequence += 1

    def get_entries(self, limit: int | None = None) -> list[dict[str, Any]]:
        with self._lock:
            entries = list(self._entries)
        if limit is None or limit >= len(entries):
            return entries
        return entries[-limit:]


class ConsoleNoiseFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        if record.levelno > logging.INFO:
            return True

        message = record.getMessage()
        if (
            record.name == "vt_franka_workspace.teleop.quest_server"
            and "Teleop control iteration failed: Controller state is stale:" in message
        ):
            return False
        if (
            record.name == "vt_franka_workspace.publishers.state_bridge"
            and "State bridge iteration failed: Controller state is stale:" in message
        ):
            return False
        if (
            record.name == "vt_franka_workspace.sensors.orbbec.recorder"
            and message.startswith("Orbbec RGB capture alive:")
        ):
            return False
        return True


def install_operator_logging(
    log_buffer: OperatorLogBuffer,
    *,
    suppress_console_noise: bool = True,
) -> None:
    root = logging.getLogger()

    if not any(handler is log_buffer for handler in root.handlers):
        root.addHandler(log_buffer)

    if suppress_console_noise:
        for handler in root.handlers:
            if handler is log_buffer:
                continue
            if not any(isinstance(existing_filter, ConsoleNoiseFilter) for existing_filter in handler.filters):
                handler.addFilter(ConsoleNoiseFilter())
