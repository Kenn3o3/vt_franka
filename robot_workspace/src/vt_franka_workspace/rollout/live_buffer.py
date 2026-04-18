from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class LiveSample:
    name: str
    data: Any
    metadata: dict[str, Any]
    captured_wall_time: float

    def age_sec(self) -> float:
        return max(0.0, time.time() - self.captured_wall_time)


class LiveSampleBuffer:
    def __init__(self, name: str) -> None:
        self.name = name
        self._lock = threading.Lock()
        self._latest: LiveSample | None = None
        self._sample_count = 0

    def update(self, data: Any, metadata: dict[str, Any] | None = None, captured_wall_time: float | None = None) -> LiveSample:
        sample = LiveSample(
            name=self.name,
            data=data,
            metadata=dict(metadata or {}),
            captured_wall_time=float(captured_wall_time if captured_wall_time is not None else time.time()),
        )
        with self._lock:
            self._latest = sample
            self._sample_count += 1
        return sample

    def get_latest(self, max_age_sec: float | None = None) -> LiveSample:
        with self._lock:
            sample = self._latest
        if sample is None:
            raise RuntimeError(f"{self.name} sample is not available yet")
        if max_age_sec is not None and sample.age_sec() > max_age_sec:
            raise RuntimeError(f"{self.name} sample is stale: age={sample.age_sec():.3f}s")
        return sample

    def get_latest_optional(self, max_age_sec: float | None = None) -> LiveSample | None:
        try:
            return self.get_latest(max_age_sec=max_age_sec)
        except RuntimeError:
            return None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            sample = self._latest
            sample_count = self._sample_count
        return {
            "available": sample is not None,
            "age_sec": None if sample is None else sample.age_sec(),
            "sample_count": sample_count,
            "metadata": {} if sample is None else _json_safe(sample.metadata),
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
