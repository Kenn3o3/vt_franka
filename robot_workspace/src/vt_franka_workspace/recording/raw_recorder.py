from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from .session import EpisodeSessionManager


def _json_default(value: Any):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Object of type {type(value)!r} is not JSON serializable")


class JsonlStreamRecorder:
    def __init__(self, session_manager: EpisodeSessionManager, stream_name: str, record_hz: float = 0.0) -> None:
        self.session_manager = session_manager
        self.stream_name = stream_name
        self.record_hz = float(record_hz)
        self._lock = threading.Lock()
        self._last_record_time: float | None = None
        self._last_episode_dir: Path | None = None

    def record_event(self, payload: dict[str, Any], event_time: float | None = None) -> None:
        episode_dir = self.session_manager.get_active_episode_dir()
        if episode_dir is None:
            self._last_episode_dir = None
            self._last_record_time = None
            return
        event_time = float(event_time if event_time is not None else payload.get("recorded_at_wall_time", time.time()))
        if not self._should_record(episode_dir, event_time):
            return
        self._write_event(episode_dir, payload)

    def _write_event(self, episode_dir: Path, payload: dict[str, Any]) -> None:
        stream_dir = episode_dir / "streams"
        stream_dir.mkdir(parents=True, exist_ok=True)
        record = dict(payload)
        record.setdefault("recorded_at_wall_time", time.time())
        with self._lock:
            with (stream_dir / f"{self.stream_name}.jsonl").open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, default=_json_default))
                handle.write("\n")

    def record_frame(
        self,
        frame: np.ndarray,
        frame_id: str,
        metadata: dict[str, Any] | None = None,
        image_format: str = "jpg",
        extra_event_fields: dict[str, Any] | None = None,
        event_time: float | None = None,
    ) -> Path | None:
        episode_dir = self.session_manager.get_active_episode_dir()
        if episode_dir is None:
            self._last_episode_dir = None
            self._last_record_time = None
            return None
        event_time = float(event_time if event_time is not None else time.time())
        if not self._should_record(episode_dir, event_time):
            return None
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - runtime dependency for image writing
            raise RuntimeError("OpenCV is required to record image frames") from exc

        frame_dir = episode_dir / "streams" / self.stream_name
        frame_dir.mkdir(parents=True, exist_ok=True)
        frame_path = frame_dir / f"{frame_id}.{image_format}"
        success, encoded = cv2.imencode(f".{image_format}", frame)
        if not success:
            raise RuntimeError("Failed to encode frame for recording")
        frame_path.write_bytes(encoded.tobytes())
        payload = {"frame_path": frame_path.relative_to(episode_dir).as_posix()}
        if metadata:
            payload["metadata"] = metadata
        if extra_event_fields:
            payload.update(extra_event_fields)
        self._write_event(episode_dir, payload)
        return frame_path

    def _should_record(self, episode_dir: Path, event_time: float) -> bool:
        with self._lock:
            if self._last_episode_dir != episode_dir:
                self._last_episode_dir = episode_dir
                self._last_record_time = None
            if self.record_hz <= 0.0:
                self._last_record_time = event_time
                return True
            if self._last_record_time is None or event_time - self._last_record_time >= 1.0 / self.record_hz:
                self._last_record_time = event_time
                return True
            return False
