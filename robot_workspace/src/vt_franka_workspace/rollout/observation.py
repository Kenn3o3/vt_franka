from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np

from vt_franka_shared.models import ControllerState

from ..settings import RolloutPolicyInputSettings
from .live_buffer import LiveSample, LiveSampleBuffer


class ObservationAssembler:
    def __init__(
        self,
        *,
        input_settings: RolloutPolicyInputSettings,
        state_provider,
        rgb_camera_buffers: dict[str, LiveSampleBuffer] | None = None,
        gelsight_marker_buffer: LiveSampleBuffer | None = None,
        gelsight_frame_buffer: LiveSampleBuffer | None = None,
        image_format: str = "jpg",
    ) -> None:
        self.input_settings = input_settings
        self.state_provider = state_provider
        self.rgb_camera_buffers = dict(rgb_camera_buffers or {})
        self.gelsight_marker_buffer = gelsight_marker_buffer
        self.gelsight_frame_buffer = gelsight_frame_buffer
        self.image_format = image_format

    def assert_ready(self) -> tuple[bool, list[str]]:
        reasons: list[str] = []
        if self.input_settings.controller_state:
            try:
                self.state_provider(max_age_sec=self.input_settings.controller_state_max_age_sec)
            except Exception as exc:
                reasons.append(f"controller_state unavailable: {exc}")
        for role in self.input_settings.rgb_cameras:
            self._check_buffer(self.rgb_camera_buffers.get(role), self.input_settings.rgb_camera_max_age_sec, f"rgb_camera:{role}", reasons)
        if self.input_settings.gelsight_markers:
            self._check_buffer(self.gelsight_marker_buffer, self.input_settings.gelsight_max_age_sec, "gelsight_markers", reasons)
        if self.input_settings.gelsight_frame:
            self._check_buffer(self.gelsight_frame_buffer, self.input_settings.gelsight_max_age_sec, "gelsight_frame", reasons)
        return not reasons, reasons

    def assemble(self, episode_dir: Path, step_index: int) -> tuple[dict[str, Any], dict[str, Any]]:
        observation: dict[str, Any] = {}
        recorded: dict[str, Any] = {}
        if self.input_settings.controller_state:
            state = self.state_provider(max_age_sec=self.input_settings.controller_state_max_age_sec)
            if isinstance(state, ControllerState):
                state_payload = state.model_dump(mode="json")
            else:
                state_payload = dict(state)
            observation["controller_state"] = state_payload
            recorded["controller_state"] = state_payload

        for role in self.input_settings.rgb_cameras:
            sample = self._required_sample(
                self.rgb_camera_buffers.get(role),
                self.input_settings.rgb_camera_max_age_sec,
                f"rgb_camera:{role}",
            )
            stream_name = sample.name
            rel_path = self._write_frame(episode_dir, stream_name, sample, step_index)
            observation[role] = {
                "image": sample.data,
                "metadata": dict(sample.metadata),
                "captured_wall_time": sample.captured_wall_time,
            }
            recorded[role] = self._recorded_image_sample(sample, rel_path)

        if self.input_settings.gelsight_markers:
            sample = self._required_sample(self.gelsight_marker_buffer, self.input_settings.gelsight_max_age_sec, "gelsight_markers")
            observation["gelsight_markers"] = {
                "marker_locations": sample.data["marker_locations"],
                "marker_offsets": sample.data["marker_offsets"],
                "metadata": dict(sample.metadata),
                "captured_wall_time": sample.captured_wall_time,
            }
            recorded["gelsight_markers"] = {
                "captured_wall_time": sample.captured_wall_time,
                "marker_locations": _json_safe(sample.data["marker_locations"]),
                "marker_offsets": _json_safe(sample.data["marker_offsets"]),
                "metadata": _json_safe(sample.metadata),
            }

        if self.input_settings.gelsight_frame:
            sample = self._required_sample(self.gelsight_frame_buffer, self.input_settings.gelsight_max_age_sec, "gelsight_frame")
            rel_path = self._write_frame(episode_dir, "gelsight_frame", sample, step_index)
            observation["gelsight_frame"] = {
                "image": sample.data,
                "metadata": dict(sample.metadata),
                "captured_wall_time": sample.captured_wall_time,
            }
            recorded["gelsight_frame"] = self._recorded_image_sample(sample, rel_path)

        recorded["assembled_wall_time"] = time.time()
        return observation, recorded

    @staticmethod
    def _check_buffer(buffer: LiveSampleBuffer | None, max_age_sec: float, name: str, reasons: list[str]) -> None:
        if buffer is None:
            reasons.append(f"{name} buffer is not configured")
            return
        try:
            buffer.get_latest(max_age_sec=max_age_sec)
        except RuntimeError as exc:
            reasons.append(str(exc))

    @staticmethod
    def _required_sample(buffer: LiveSampleBuffer | None, max_age_sec: float, name: str) -> LiveSample:
        if buffer is None:
            raise RuntimeError(f"{name} buffer is not configured")
        return buffer.get_latest(max_age_sec=max_age_sec)

    def _write_frame(self, episode_dir: Path, stream_name: str, sample: LiveSample, step_index: int) -> str:
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - runtime dependency
            raise RuntimeError("OpenCV is required to record rollout image observations") from exc

        frame_dir = episode_dir / "streams" / stream_name
        frame_dir.mkdir(parents=True, exist_ok=True)
        frame_path = frame_dir / f"step_{step_index:06d}.{self.image_format}"
        success, encoded = cv2.imencode(f".{self.image_format}", sample.data)
        if not success:
            raise RuntimeError(f"Failed to encode {stream_name} frame")
        frame_path.write_bytes(encoded.tobytes())
        return frame_path.relative_to(episode_dir).as_posix()

    @staticmethod
    def _recorded_image_sample(sample: LiveSample, rel_path: str) -> dict[str, Any]:
        image = sample.data
        height = int(image.shape[0]) if hasattr(image, "shape") and len(image.shape) >= 2 else 0
        width = int(image.shape[1]) if hasattr(image, "shape") and len(image.shape) >= 2 else 0
        return {
            "captured_wall_time": sample.captured_wall_time,
            "frame_path": rel_path,
            "frame_width": width,
            "frame_height": height,
            "metadata": _json_safe(sample.metadata),
        }


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value
