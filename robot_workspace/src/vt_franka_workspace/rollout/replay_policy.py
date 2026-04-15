from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..settings import TeleopSettings


@dataclass
class ReplayEpisode:
    timestamps: np.ndarray
    target_tcp: np.ndarray
    gripper_closed: np.ndarray
    hz: float


def load_replay_episode(path: str | Path) -> ReplayEpisode:
    path = Path(path)
    if path.is_dir():
        path = path / "aligned_episode.npz"
    if not path.exists():
        raise FileNotFoundError(f"Replay source does not exist: {path}")

    data = np.load(path, allow_pickle=True)
    timestamps = np.asarray(data["timestamps"], dtype=np.float64)
    target_tcp = np.asarray(data["teleop_target_tcp"], dtype=np.float64)
    if target_tcp.ndim != 2 or target_tcp.shape[1] != 7:
        raise RuntimeError(f"Expected teleop_target_tcp to have shape (N, 7), got {target_tcp.shape}")

    if "teleop_gripper_closed" in data:
        gripper_closed = np.asarray(data["teleop_gripper_closed"], dtype=bool)
    else:
        gripper_closed = np.zeros((len(timestamps),), dtype=bool)

    hz = infer_sync_hz(timestamps)
    return ReplayEpisode(
        timestamps=timestamps,
        target_tcp=target_tcp,
        gripper_closed=gripper_closed,
        hz=hz,
    )


def infer_sync_hz(timestamps: np.ndarray) -> float:
    if len(timestamps) < 2:
        return 0.0
    deltas = np.diff(timestamps)
    positive = deltas[deltas > 1e-6]
    if len(positive) == 0:
        return 0.0
    return float(1.0 / np.median(positive))


def build_replay_policy(
    *,
    episode_dir: str | Path | None = None,
    hz: float | None = None,
    speed_scale: float = 1.0,
    skip_gripper: bool = False,
    teleop_settings: TeleopSettings | None = None,
    **_: Any,
):
    if episode_dir is None:
        raise ValueError("Replay policy requires --episode-dir")
    if speed_scale <= 0.0:
        raise ValueError("speed_scale must be positive")

    episode = load_replay_episode(episode_dir)
    if len(episode.timestamps) == 0:
        raise ValueError("Replay episode is empty")

    teleop_settings = teleop_settings or TeleopSettings()
    relative_timestamps = _build_relative_timestamps(episode, hz=hz)
    effective_hz = hz if hz is not None and hz > 0.0 else episode.hz
    if effective_hz <= 0.0:
        effective_hz = 1.0

    start_monotonic: float | None = None
    last_gripper_closed: bool | None = None
    finished = False
    final_index = len(relative_timestamps) - 1

    def replay_policy(observation: dict) -> dict:
        nonlocal start_monotonic, last_gripper_closed, finished
        del observation

        now = time.monotonic()
        if start_monotonic is None:
            start_monotonic = now
        elapsed = (now - start_monotonic) * speed_scale

        index = int(np.searchsorted(relative_timestamps, elapsed, side="right") - 1)
        index = min(max(index, 0), final_index)

        action: dict[str, Any] = {
            "target_tcp": episode.target_tcp[index].astype(float).tolist(),
        }

        if not skip_gripper:
            gripper_closed = bool(episode.gripper_closed[index])
            if last_gripper_closed is None or gripper_closed != last_gripper_closed:
                action["gripper_velocity"] = teleop_settings.gripper_velocity
                action["gripper_force_limit"] = teleop_settings.grasp_force
                if gripper_closed:
                    action["gripper_closed"] = True
                else:
                    action["gripper_width"] = teleop_settings.max_gripper_width
                last_gripper_closed = gripper_closed

        if not finished and elapsed >= relative_timestamps[-1]:
            action["terminate"] = True
            finished = True

        return action

    replay_policy.__vt_franka_control_hz__ = effective_hz
    replay_policy.__vt_franka_max_duration_sec__ = relative_timestamps[-1] / speed_scale + 1.0 / effective_hz
    return replay_policy


build_replay_policy.__vt_franka_policy_factory__ = True


def _build_relative_timestamps(episode: ReplayEpisode, hz: float | None) -> np.ndarray:
    if hz is not None and hz > 0.0:
        return np.arange(len(episode.timestamps), dtype=np.float64) / hz

    relative = np.asarray(episode.timestamps, dtype=np.float64)
    relative = relative - relative[0]
    if len(relative) == 1:
        return np.zeros((1,), dtype=np.float64)
    return relative
