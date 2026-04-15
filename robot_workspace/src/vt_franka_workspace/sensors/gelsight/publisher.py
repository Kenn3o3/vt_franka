from __future__ import annotations

import math
import time
from threading import Event
from typing import Callable

import numpy as np
import requests

from vt_franka_shared.models import Arrow, TactileSensorMessage
from vt_franka_shared.timing import precise_sleep

from ...publishers.quest_udp import QuestUdpPublisher
from ...recording.raw_recorder import JsonlStreamRecorder
from ...settings import GelsightSettings, RecordingSettings
from .tracker import GelsightMarkerTracker


class GelsightPublisher:
    def __init__(
        self,
        settings: GelsightSettings,
        quest_publisher: QuestUdpPublisher,
        marker_recorder: JsonlStreamRecorder | None = None,
        frame_recorder: JsonlStreamRecorder | None = None,
        image_format: str = "jpg",
        gripper_status_provider: Callable[[], dict[str, bool]] | None = None,
    ) -> None:
        self.settings = settings
        self.quest_publisher = quest_publisher
        self.marker_recorder = marker_recorder
        self.frame_recorder = frame_recorder
        self.image_format = image_format
        self.gripper_status_provider = gripper_status_provider
        self.tracker = GelsightMarkerTracker()
        self._base_marker_motion: np.ndarray | None = None
        self._latency_counter = 0
        self._previous_gripper_state = {
            "left_gripper_stable_closed": False,
            "left_gripper_stable_open": True,
        }
        self._frames_seen = 0

    def run(self, stop_event: Event | None = None) -> None:
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - runtime dependency
            raise RuntimeError("OpenCV is required for the GelSight publisher") from exc

        cap = cv2.VideoCapture(self.settings.camera_index)
        if not cap.isOpened():
            raise RuntimeError(f"Unable to open GelSight camera index {self.settings.camera_index}")
        time.sleep(1.0)

        period = 1.0 / self.settings.fps
        try:
            while stop_event is None or not stop_event.is_set():
                loop_start = time.time()
                ok, frame = cap.read()
                if not ok:
                    precise_sleep(period)
                    continue

                resized = cv2.resize(frame, (self.settings.width, self.settings.height))
                initial_markers, marker_offsets = self.tracker.process_frame(resized)
                initial_markers_norm, marker_offsets_norm = self.tracker.normalize_markers(
                    initial_markers.copy(),
                    marker_offsets.copy(),
                    width=self.settings.width,
                    height=self.settings.height,
                )

                gripper_state = self._fetch_gripper_state()
                marker_offsets_vis = self._latency_match(marker_offsets_norm.copy(), gripper_state)
                tactile = self._build_tactile_message(initial_markers_norm.copy(), marker_offsets_vis)
                self.quest_publisher.publish_tactile(tactile)

                captured_wall_time = time.time()
                if self.marker_recorder is not None:
                    self.marker_recorder.record_event(
                        {
                            "captured_wall_time": captured_wall_time,
                            "marker_locations": initial_markers_norm[:, :2],
                            "marker_offsets": marker_offsets_norm,
                        },
                        event_time=captured_wall_time,
                    )
                if self.frame_recorder is not None and self.settings.save_frames:
                    self.frame_recorder.record_frame(
                        resized,
                        frame_id=f"{captured_wall_time:.6f}".replace(".", "_"),
                        metadata={"captured_wall_time": captured_wall_time},
                        image_format=self.image_format,
                        event_time=captured_wall_time,
                    )

                self._frames_seen += 1
                precise_sleep(max(0.0, period - (time.time() - loop_start)))
        finally:
            cap.release()

    def _fetch_gripper_state(self) -> dict[str, bool]:
        if self.gripper_status_provider is not None:
            try:
                return self.gripper_status_provider()
            except Exception:
                return dict(self._previous_gripper_state)
        url = f"http://{self.settings.teleop_status_host}:{self.settings.teleop_status_port}/get_current_gripper_state"
        try:
            response = requests.get(url, timeout=0.1)
            response.raise_for_status()
            return response.json()
        except Exception:
            return dict(self._previous_gripper_state)

    @property
    def frames_seen(self) -> int:
        return self._frames_seen

    def _latency_match(self, marker_offsets: np.ndarray, gripper_state: dict[str, bool]) -> np.ndarray:
        if (
            not self._previous_gripper_state["left_gripper_stable_open"]
            and gripper_state.get("left_gripper_stable_open", False)
        ):
            self._base_marker_motion = None
        elif (
            not self._previous_gripper_state["left_gripper_stable_closed"]
            and gripper_state.get("left_gripper_stable_closed", False)
        ):
            self._latency_counter = self.settings.vis_latency_steps

        if self._latency_counter > 0:
            self._latency_counter -= 1
            if self._latency_counter == 0:
                self._base_marker_motion = marker_offsets.copy()

        self._previous_gripper_state = {
            "left_gripper_stable_closed": bool(gripper_state.get("left_gripper_stable_closed", False)),
            "left_gripper_stable_open": bool(gripper_state.get("left_gripper_stable_open", True)),
        }

        if self._base_marker_motion is not None:
            marker_offsets = marker_offsets - self._base_marker_motion
        return marker_offsets

    def _build_tactile_message(self, initial_markers: np.ndarray, marker_offsets: np.ndarray) -> TactileSensorMessage:
        initial_markers[:, :2] = initial_markers[:, :2] - 0.5
        initial_markers *= 0.25
        marker_offsets *= 2.0
        z_offset = 0.1
        rotation = math.radians(self.settings.marker_vis_rotation_deg)
        rotation_matrix = np.array(
            [[math.cos(rotation), -math.sin(rotation)], [math.sin(rotation), math.cos(rotation)]],
            dtype=np.float32,
        )
        initial_markers[:, :2] = initial_markers[:, :2] @ rotation_matrix.T
        marker_offsets[:, :2] = marker_offsets[:, :2] @ rotation_matrix.T

        arrows = []
        for marker, offset in zip(initial_markers, marker_offsets):
            start = [float(marker[0]), float(marker[1]), z_offset]
            end = [float(marker[0] + offset[0]), float(marker[1] + offset[1]), z_offset]
            arrows.append(Arrow(start=start, end=end))
        return TactileSensorMessage(device_id=self.settings.camera_name, arrows=arrows)
