from __future__ import annotations

import math
import os
import subprocess
import time
from pathlib import Path
from threading import Event
from typing import Callable

import numpy as np
import requests

from vt_franka_shared.models import Arrow, TactileSensorMessage
from vt_franka_shared.timing import precise_sleep

from ...publishers.quest_udp import QuestUdpPublisher
from ...rollout.live_buffer import LiveSampleBuffer
from ...recording.raw_recorder import JsonlStreamRecorder
from ...settings import GelsightSettings
from .tracker import GelsightMarkerTracker


class GelsightPublisher:
    def __init__(
        self,
        settings: GelsightSettings,
        quest_publisher: QuestUdpPublisher,
        marker_recorder: JsonlStreamRecorder | None = None,
        frame_recorder: JsonlStreamRecorder | None = None,
        marker_buffer: LiveSampleBuffer | None = None,
        frame_buffer: LiveSampleBuffer | None = None,
        image_format: str = "jpg",
        gripper_status_provider: Callable[[], dict[str, bool]] | None = None,
    ) -> None:
        self.settings = settings
        self.quest_publisher = quest_publisher
        self.marker_recorder = marker_recorder
        self.frame_recorder = frame_recorder
        self.marker_buffer = marker_buffer
        self.frame_buffer = frame_buffer
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

        source = self._resolve_capture_source()
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError(f"Unable to open GelSight camera source {source!r}")
        self._configure_capture(cap)
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
                if self.settings.quest_stream.enabled:
                    overlay = self._build_overlay_frame(
                        resized,
                        initial_markers_norm[:, :2].copy(),
                        marker_offsets_vis.copy(),
                    )
                    self.quest_publisher.publish_image(overlay, self.settings.quest_stream)

                captured_wall_time = time.time()
                if self.marker_buffer is not None:
                    self.marker_buffer.update(
                        {
                            "marker_locations": initial_markers_norm[:, :2].copy(),
                            "marker_offsets": marker_offsets_norm.copy(),
                        },
                        metadata={"camera_name": self.settings.camera_name},
                        captured_wall_time=captured_wall_time,
                    )
                if self.frame_buffer is not None:
                    self.frame_buffer.update(
                        resized.copy(),
                        metadata={"camera_name": self.settings.camera_name},
                        captured_wall_time=captured_wall_time,
                    )
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

    def _resolve_capture_source(self) -> str | int:
        if self.settings.camera_path:
            return self.settings.camera_path

        candidates = _list_v4l_video_capture_candidates()
        matched = _filter_candidates(
            candidates,
            name_contains=self.settings.device_name_contains,
            serial_number=self.settings.device_serial_number,
        )
        if matched:
            return matched[0]["device_path"]
        return self.settings.camera_index

    def _configure_capture(self, cap) -> None:
        try:
            import cv2
        except ImportError:
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.settings.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.settings.height)
        cap.set(cv2.CAP_PROP_FPS, self.settings.fps)
        if self.settings.exposure is not None:
            cap.set(cv2.CAP_PROP_EXPOSURE, self.settings.exposure)
        if self.settings.contrast is not None:
            cap.set(cv2.CAP_PROP_CONTRAST, self.settings.contrast)

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

    def _build_overlay_frame(
        self,
        frame: np.ndarray,
        marker_locations_norm: np.ndarray,
        marker_offsets_norm: np.ndarray,
    ) -> np.ndarray:
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - runtime dependency
            raise RuntimeError("OpenCV is required for GelSight overlay visualization") from exc

        overlay = frame.copy()
        height, width = overlay.shape[:2]
        marker_locations = marker_locations_norm.copy()
        marker_offsets = marker_offsets_norm.copy()
        marker_locations[:, 0] *= width
        marker_locations[:, 1] *= height
        marker_offsets[:, 0] *= width * self.settings.quest_overlay_arrow_scale
        marker_offsets[:, 1] *= height * self.settings.quest_overlay_arrow_scale
        for marker, offset in zip(marker_locations, marker_offsets):
            start = (int(round(marker[0])), int(round(marker[1])))
            end = (
                int(round(marker[0] + offset[0])),
                int(round(marker[1] + offset[1])),
            )
            cv2.circle(overlay, start, 2, (80, 255, 80), thickness=-1)
            cv2.arrowedLine(overlay, start, end, (40, 80, 255), thickness=1, tipLength=0.25)
        return overlay


def _filter_candidates(
    candidates: list[dict[str, str]],
    *,
    name_contains: str,
    serial_number: str,
) -> list[dict[str, str]]:
    filtered = candidates
    if name_contains:
        needle = name_contains.lower()
        filtered = [candidate for candidate in filtered if needle in candidate["name"].lower()]
    if serial_number:
        filtered = [candidate for candidate in filtered if candidate["serial_number"] == serial_number]
    return filtered


def _list_v4l_video_capture_candidates() -> list[dict[str, str]]:
    candidates: list[dict[str, str]] = []
    by_id_dir = Path("/dev/v4l/by-id")
    if by_id_dir.exists():
        for symlink in sorted(by_id_dir.iterdir()):
            try:
                resolved = symlink.resolve(strict=True)
            except FileNotFoundError:
                continue
            if not resolved.name.startswith("video"):
                continue
            info = _query_v4l_device(resolved)
            if info is None or not info["video_capture"]:
                continue
            info["stable_path"] = str(symlink)
            info["device_path"] = str(symlink)
            candidates.append(info)

    if candidates:
        return _deduplicate_candidates(candidates)

    for device in sorted(Path("/dev").glob("video*")):
        info = _query_v4l_device(device)
        if info is None or not info["video_capture"]:
            continue
        info["stable_path"] = str(device)
        info["device_path"] = str(device)
        candidates.append(info)
    return _deduplicate_candidates(candidates)


def _deduplicate_candidates(candidates: list[dict[str, str]]) -> list[dict[str, str]]:
    deduped: list[dict[str, str]] = []
    seen: set[str] = set()
    for candidate in candidates:
        key = os.path.realpath(candidate["device_path"])
        if key in seen:
            continue
        seen.add(key)
        deduped.append(candidate)
    return deduped


def _query_v4l_device(device: Path) -> dict[str, str] | None:
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", str(device), "--all"],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except Exception:
        return None
    if result.returncode != 0:
        return None

    stdout = result.stdout
    video_capture = "Video Capture" in stdout and "Metadata Capture" not in _device_caps_section(stdout)
    card_type = _extract_v4l_value(stdout, "Card type") or _extract_v4l_value(stdout, "Model") or device.name
    serial_number = _extract_v4l_value(stdout, "Serial") or ""
    return {
        "name": card_type,
        "serial_number": serial_number,
        "video_capture": video_capture,
    }


def _device_caps_section(text: str) -> str:
    marker = "Device Caps"
    index = text.find(marker)
    if index == -1:
        return text
    return text[index : index + 256]


def _extract_v4l_value(text: str, key: str) -> str | None:
    prefix = f"{key:<17}:"
    for line in text.splitlines():
        if line.strip().startswith(f"{key: <17}:".strip()):
            _, _, value = line.partition(":")
            return value.strip()
        if line.startswith(prefix):
            _, _, value = line.partition(":")
            return value.strip()
    return None
