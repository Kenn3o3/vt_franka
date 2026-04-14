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
    def __init__(self, session_manager: EpisodeSessionManager, stream_name: str) -> None:
        self.session_manager = session_manager
        self.stream_name = stream_name
        self._lock = threading.Lock()

    def record_event(self, payload: dict[str, Any]) -> None:
        episode_dir = self.session_manager.get_active_episode_dir()
        if episode_dir is None:
            return
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
    ) -> Path | None:
        episode_dir = self.session_manager.get_active_episode_dir()
        if episode_dir is None:
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
        self.record_event(payload)
        return frame_path
