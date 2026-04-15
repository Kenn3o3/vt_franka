from __future__ import annotations

import importlib
import logging
import time
from threading import Event
from typing import Any

from ...recording.raw_recorder import JsonlStreamRecorder
from ...settings import OrbbecSettings
from .frame_decoder import decode_color_frame

logger = logging.getLogger(__name__)


class OrbbecRgbRecorder:
    def __init__(
        self,
        settings: OrbbecSettings,
        recorder: JsonlStreamRecorder | None = None,
        image_format: str = "jpg",
        sdk_module: Any | None = None,
    ) -> None:
        self.settings = settings
        self.recorder = recorder
        self.image_format = image_format
        self._sdk_module = sdk_module
        self._device_name = settings.camera_name
        self._device_serial_number = settings.serial_number
        self._frames_seen = 0
        self._last_status_log_time = time.monotonic()

    def run(self, stop_event: Event | None = None) -> None:
        sdk = self._load_sdk()
        pipeline = self._open_pipeline(sdk)
        try:
            while stop_event is None or not stop_event.is_set():
                frames = pipeline.wait_for_frames(self.settings.frame_timeout_ms)
                if frames is None:
                    continue
                color_frame = frames.get_color_frame()
                if color_frame is None:
                    continue
                self._record_color_frame(color_frame)
        finally:
            pipeline.stop()

    def _load_sdk(self) -> Any:
        if self._sdk_module is not None:
            return self._sdk_module
        try:
            return importlib.import_module("pyorbbecsdk")
        except ImportError as exc:  # pragma: no cover - depends on local SDK install
            raise RuntimeError(
                "pyorbbecsdk is not installed. Install the Orbbec Python SDK in the workspace environment first."
            ) from exc

    def _open_pipeline(self, sdk: Any) -> Any:
        context = sdk.Context()
        device_list = context.query_devices()
        device_count = int(device_list.get_count())
        if device_count == 0:
            raise RuntimeError("No Orbbec devices found")

        if device_count > 1 and not self.settings.serial_number:
            logger.warning("Multiple Orbbec devices detected; using the first one because no serial_number was configured")

        selected_device = None
        for index in range(device_count):
            device = _device_at_index(device_list, index)
            device_info = device.get_device_info()
            serial_number = _safe_call(device_info, "get_serial_number")
            if self.settings.serial_number and serial_number != self.settings.serial_number:
                continue
            selected_device = device
            self._device_name = _safe_call(device_info, "get_name") or self.settings.camera_name
            self._device_serial_number = serial_number or self.settings.serial_number
            break

        if selected_device is None:
            raise RuntimeError(f"Configured Orbbec serial_number was not found: {self.settings.serial_number}")

        pipeline = sdk.Pipeline(selected_device)
        config = sdk.Config()
        profile_list = pipeline.get_stream_profile_list(sdk.OBSensorType.COLOR_SENSOR)
        profile = self._select_color_profile(profile_list, sdk)
        config.enable_stream(profile)
        pipeline.start(config)
        logger.info(
            "Started Orbbec RGB stream: name=%s serial=%s format=%s width=%s height=%s fps=%s",
            self._device_name,
            self._device_serial_number or "<unknown>",
            _safe_call(profile, "get_format"),
            _safe_call(profile, "get_width"),
            _safe_call(profile, "get_height"),
            _safe_call(profile, "get_fps"),
        )
        return pipeline

    def _select_color_profile(self, profile_list: Any, sdk: Any) -> Any:
        requested_format = getattr(sdk.OBFormat, self.settings.color_format.upper(), None)
        if requested_format is None:
            raise RuntimeError(f"Unsupported Orbbec color_format in config: {self.settings.color_format}")

        try:
            return profile_list.get_video_stream_profile(
                self.settings.color_width,
                self.settings.color_height,
                requested_format,
                self.settings.color_fps,
            )
        except Exception as exc:
            logger.warning(
                "Requested Orbbec profile %sx%s %s @ %s FPS unavailable, falling back to default profile: %s",
                self.settings.color_width,
                self.settings.color_height,
                self.settings.color_format,
                self.settings.color_fps,
                exc,
            )
            return profile_list.get_default_video_stream_profile()

    def _record_color_frame(self, color_frame: Any) -> None:
        try:
            image = decode_color_frame(color_frame)
        except ValueError as exc:
            logger.warning("Skipping Orbbec frame due to unsupported conversion: %s", exc)
            return

        captured_wall_time = time.time()
        sequence_id = _safe_call(color_frame, "get_index")
        payload = {
            "camera_name": self._device_name,
            "serial_number": self._device_serial_number,
            "captured_wall_time": captured_wall_time,
            "device_timestamp_us": _safe_call(color_frame, "get_timestamp_us") or _safe_call(color_frame, "get_timestamp"),
            "system_timestamp_us": _safe_call(color_frame, "get_system_timestamp_us"),
            "sequence_id": sequence_id,
            "frame_width": int(image.shape[1]),
            "frame_height": int(image.shape[0]),
            "color_format": str(_safe_call(color_frame, "get_format") or self.settings.color_format),
        }

        if self.recorder is not None:
            if self.settings.save_frames:
                frame_id = _build_frame_id(captured_wall_time, sequence_id)
                self.recorder.record_frame(
                    image,
                    frame_id=frame_id,
                    image_format=self.image_format,
                    extra_event_fields=payload,
                    event_time=captured_wall_time,
                )
            else:
                self.recorder.record_event(payload, event_time=captured_wall_time)

        self._frames_seen += 1
        now = time.monotonic()
        if now - self._last_status_log_time >= 2.0:
            active_episode = self.recorder.session_manager.get_active_episode_dir() if self.recorder is not None else None
            logger.info(
                "Orbbec RGB capture alive: frames=%d active_episode=%s",
                self._frames_seen,
                str(active_episode) if active_episode is not None else "<none>",
            )
            self._last_status_log_time = now

    @property
    def frames_seen(self) -> int:
        return self._frames_seen


def _build_frame_id(captured_wall_time: float, sequence_id: Any) -> str:
    base = f"{captured_wall_time:.6f}".replace(".", "_")
    if sequence_id is None:
        return base
    return f"{base}_{sequence_id}"


def _device_at_index(device_list: Any, index: int) -> Any:
    try:
        return device_list[index]
    except Exception:
        return device_list.get_device_by_index(index)


def _safe_call(obj: Any, method_name: str) -> Any | None:
    method = getattr(obj, method_name, None)
    if method is None:
        return None
    try:
        return method()
    except Exception:
        return None
