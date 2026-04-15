from __future__ import annotations

import logging
import time
from threading import Event, Lock, Thread
from typing import Any

from vt_franka_shared.models import ControllerState
from vt_franka_shared.timing import precise_sleep

from ..controller.client import ControllerClient

LOGGER = logging.getLogger(__name__)


class ControllerStateMonitor:
    def __init__(self, controller: ControllerClient, poll_hz: float = 60.0, warning_interval_sec: float = 5.0) -> None:
        self.controller = controller
        self.poll_hz = poll_hz
        self.warning_interval_sec = warning_interval_sec
        self._running = Event()
        self._thread: Thread | None = None
        self._lock = Lock()
        self._latest_state: ControllerState | None = None
        self._latest_received_wall_time: float | None = None
        self._last_error: str | None = None
        self._last_warning_wall_time = 0.0
        self._sample_count = 0
        self._failure_count = 0
        self._max_gap_sec = 0.0

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread = Thread(target=self._loop, name="controller-state-monitor", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def get_state(self, max_age_sec: float | None = None) -> ControllerState:
        with self._lock:
            state = self._latest_state
            received_wall_time = self._latest_received_wall_time
            last_error = self._last_error
        if state is None or received_wall_time is None:
            raise RuntimeError(last_error or "Controller state is not available yet")
        if max_age_sec is not None and time.time() - received_wall_time > max_age_sec:
            raise RuntimeError(f"Controller state is stale: age={time.time() - received_wall_time:.3f}s")
        return state

    def get_state_optional(self, max_age_sec: float | None = None) -> ControllerState | None:
        try:
            return self.get_state(max_age_sec=max_age_sec)
        except RuntimeError:
            return None

    def is_healthy(self, max_age_sec: float = 2.0) -> bool:
        return self.get_state_optional(max_age_sec=max_age_sec) is not None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            received_wall_time = self._latest_received_wall_time
            sample_count = self._sample_count
            failure_count = self._failure_count
            max_gap_sec = self._max_gap_sec
            last_error = self._last_error
        age_sec = None if received_wall_time is None else max(0.0, time.time() - received_wall_time)
        return {
            "healthy": received_wall_time is not None and age_sec is not None,
            "age_sec": age_sec,
            "sample_count": sample_count,
            "failure_count": failure_count,
            "max_gap_sec": max_gap_sec,
            "last_error": last_error,
        }

    def _loop(self) -> None:
        period = 1.0 / max(self.poll_hz, 1e-6)
        previous_received_wall_time: float | None = None
        while self._running.is_set():
            try:
                state = self.controller.get_state()
                received_wall_time = time.time()
                with self._lock:
                    if previous_received_wall_time is not None:
                        self._max_gap_sec = max(self._max_gap_sec, received_wall_time - previous_received_wall_time)
                    self._latest_state = state
                    self._latest_received_wall_time = received_wall_time
                    self._sample_count += 1
                    self._last_error = None
                previous_received_wall_time = received_wall_time
            except Exception as exc:
                now = time.time()
                with self._lock:
                    self._failure_count += 1
                    self._last_error = str(exc)
                if now - self._last_warning_wall_time >= self.warning_interval_sec:
                    LOGGER.warning("Controller state monitor failed: %s", exc)
                    self._last_warning_wall_time = now
            precise_sleep(period)
