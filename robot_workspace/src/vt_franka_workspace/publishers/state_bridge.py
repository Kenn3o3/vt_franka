from __future__ import annotations

import time
from threading import Event, Thread

from vt_franka_shared.timing import precise_sleep

from ..controller.client import ControllerClient
from ..recording.raw_recorder import JsonlStreamRecorder
from ..ros.state_publisher import Ros2StatePublisher
from ..settings import QuestFeedbackSettings
from .quest_udp import QuestUdpPublisher


class StateBridge:
    def __init__(
        self,
        controller: ControllerClient,
        quest_publisher: QuestUdpPublisher,
        settings: QuestFeedbackSettings,
        recorder: JsonlStreamRecorder | None = None,
        ros_publisher: Ros2StatePublisher | None = None,
    ) -> None:
        self.controller = controller
        self.quest_publisher = quest_publisher
        self.settings = settings
        self.recorder = recorder
        self.ros_publisher = ros_publisher
        self._running = Event()
        self._thread: Thread | None = None

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread = Thread(target=self._loop, name="state-bridge-loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.ros_publisher is not None:
            self.ros_publisher.shutdown()

    def _loop(self) -> None:
        period = 1.0 / self.settings.state_publish_hz
        while self._running.is_set():
            state = self.controller.get_state()
            self.quest_publisher.publish_robot_state(state)
            if self.ros_publisher is not None:
                self.ros_publisher.publish_controller_state(state)
            if self.recorder is not None:
                self.recorder.record_event(
                    {
                        "source_wall_time": state.wall_time,
                        "source_monotonic_time": state.monotonic_time,
                        "received_wall_time": time.time(),
                        "state": state.model_dump(mode="json"),
                    }
                )
            precise_sleep(period)

