from __future__ import annotations

import logging
from threading import Event, Thread

from vt_franka_shared.timing import precise_sleep

from ..collect.controller_state import ControllerStateMonitor
from ..recording.raw_recorder import JsonlStreamRecorder

LOGGER = logging.getLogger(__name__)


class ControllerStateRecorderLoop:
    def __init__(
        self,
        state_monitor: ControllerStateMonitor,
        recorder: JsonlStreamRecorder,
        *,
        record_hz: float,
        max_age_sec: float,
    ) -> None:
        self.state_monitor = state_monitor
        self.recorder = recorder
        self.record_hz = float(record_hz)
        self.max_age_sec = float(max_age_sec)
        self._running = Event()
        self._thread: Thread | None = None

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread = Thread(target=self._loop, name="auto-collect-state-recorder", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        period = 1.0 / max(self.record_hz, 1e-6)
        while self._running.is_set():
            try:
                state = self.state_monitor.get_state(max_age_sec=self.max_age_sec)
            except RuntimeError as exc:
                LOGGER.debug("Auto-collect state recorder waiting for first controller state: %s", exc)
                precise_sleep(min(period, 0.05))
                continue
            self.recorder.record_event(
                {
                    "source_wall_time": state.wall_time,
                    "source_monotonic_time": state.monotonic_time,
                    "state": state.model_dump(mode="json"),
                },
                event_time=state.wall_time,
            )
            precise_sleep(period)
