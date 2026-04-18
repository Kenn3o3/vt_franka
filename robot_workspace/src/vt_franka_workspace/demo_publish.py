from __future__ import annotations

import logging
import time
from collections import deque
from threading import Event, Thread

from .collect.controller_state import ControllerStateMonitor
from .publishers.state_bridge import StateBridge
from .settings import WorkspaceSettings
from .sensors.gelsight.publisher import GelsightPublisher

LOGGER = logging.getLogger(__name__)


class DemoPublisher:
    def __init__(self, settings: WorkspaceSettings, controller, calibration, quest_publisher) -> None:
        self.settings = settings
        self.controller = controller
        self.calibration = calibration
        self.quest_publisher = quest_publisher
        self.state_monitor = ControllerStateMonitor(
            controller,
            poll_hz=settings.quest_feedback.state_publish_hz,
        )
        self.state_bridge: StateBridge | None = None
        self.gelsight_thread: Thread | None = None
        self.gelsight_stop_event = Event()
        self.gelsight_publisher: GelsightPublisher | None = None
        self._gripper_width_history = deque(maxlen=settings.teleop.gripper_stability_window)
        self._last_force = 0.0

    def run(self) -> None:
        self.state_monitor.start()
        state_provider = lambda: self.state_monitor.get_state(max_age_sec=2.0)
        self.state_bridge = StateBridge(
            self.controller,
            self.quest_publisher,
            self.settings.quest_feedback,
            state_provider=state_provider,
        )
        self.state_bridge.start()

        if self.settings.gelsight.enabled:
            self.gelsight_publisher = GelsightPublisher(
                self.settings.gelsight,
                self.quest_publisher,
                image_format=self.settings.recording.image_format,
                gripper_status_provider=self._get_gripper_status,
            )
            self.gelsight_thread = Thread(target=self._run_gelsight, name="demo-gelsight", daemon=True)
            self.gelsight_thread.start()

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            LOGGER.info("Demo publisher interrupted by user")
        finally:
            self.stop()

    def stop(self) -> None:
        if self.state_bridge is not None:
            self.state_bridge.stop()
        self.state_monitor.stop()
        self.gelsight_stop_event.set()
        if self.gelsight_thread is not None:
            self.gelsight_thread.join(timeout=2.0)

    def _run_gelsight(self) -> None:
        assert self.gelsight_publisher is not None
        self.gelsight_publisher.run(stop_event=self.gelsight_stop_event)

    def _get_gripper_status(self) -> dict[str, bool]:
        state = self.state_monitor.get_state(max_age_sec=2.0)
        self._last_force = state.gripper_force
        self._gripper_width_history.append(state.gripper_width)
        stable_open = self._is_gripper_stable_open()
        stable_closed = self._is_gripper_stable_closed()
        return {
            "left_gripper_stable_closed": stable_closed,
            "right_gripper_stable_closed": False,
            "left_gripper_stable_open": stable_open,
            "right_gripper_stable_open": True,
        }

    def _is_gripper_stable_open(self) -> bool:
        if self._last_force >= self.settings.teleop.gripper_force_open_threshold:
            return False
        if len(self._gripper_width_history) < self.settings.teleop.gripper_stability_window:
            return False
        width_variation = max(self._gripper_width_history) - min(self._gripper_width_history)
        return width_variation < self.settings.teleop.gripper_width_vis_precision

    def _is_gripper_stable_closed(self) -> bool:
        if self._last_force < self.settings.teleop.gripper_force_close_threshold:
            return False
        if len(self._gripper_width_history) < self.settings.teleop.gripper_stability_window:
            return False
        width_variation = max(self._gripper_width_history) - min(self._gripper_width_history)
        return width_variation < self.settings.teleop.gripper_width_vis_precision
