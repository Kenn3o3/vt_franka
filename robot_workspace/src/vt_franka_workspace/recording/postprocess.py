from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def _timestamp_array(records: list[dict[str, Any]], timestamp_key: str) -> np.ndarray:
    if not records:
        return np.empty((0,), dtype=np.float64)
    return np.array([float(item[timestamp_key]) for item in records], dtype=np.float64)


def _latest_record(
    records: list[dict[str, Any]],
    record_timestamps: np.ndarray,
    timestamp: float,
) -> tuple[dict[str, Any] | None, float]:
    if record_timestamps.size == 0:
        return None, np.nan
    index = int(np.searchsorted(record_timestamps, timestamp, side="right") - 1)
    if index < 0:
        return None, np.nan
    return records[index], float(record_timestamps[index])


def _next_record(
    records: list[dict[str, Any]],
    record_timestamps: np.ndarray,
    timestamp: float,
) -> tuple[dict[str, Any] | None, float]:
    if record_timestamps.size == 0:
        return None, np.nan
    index = int(np.searchsorted(record_timestamps, timestamp, side="right"))
    if index >= len(records):
        return None, np.nan
    return records[index], float(record_timestamps[index])


def _float_matrix(values: list[list[float]], width: int) -> np.ndarray:
    if not values:
        return np.empty((0, width), dtype=np.float64)
    return np.asarray(values, dtype=np.float64)


def align_episode(
    episode_dir: str | Path,
    target_hz: float = 24.0,
    *,
    max_action_lead_sec: float | None = None,
) -> Path:
    episode_dir = Path(episode_dir)
    streams_dir = episode_dir / "streams"
    controller = _read_jsonl(streams_dir / "controller_state.jsonl")
    teleop = _read_jsonl(streams_dir / "teleop_commands.jsonl")
    gelsight = _read_jsonl(streams_dir / "gelsight_markers.jsonl")
    orbbec = _read_jsonl(streams_dir / "orbbec_rgb.jsonl")

    if not controller:
        raise RuntimeError(f"No controller_state stream found in {episode_dir}")

    controller_times = _timestamp_array(controller, "source_wall_time")
    teleop_times = _timestamp_array(teleop, "source_wall_time")
    gelsight_times = _timestamp_array(gelsight, "captured_wall_time")
    orbbec_times = _timestamp_array(orbbec, "captured_wall_time")
    start_time = controller_times[0]
    end_time = controller_times[-1]
    step = 1.0 / target_hz
    action_horizon_sec = step if max_action_lead_sec is None else float(max_action_lead_sec)
    if action_horizon_sec <= 0.0:
        raise ValueError("max_action_lead_sec must be positive")
    grid = np.arange(start_time, end_time + step * 0.5, step, dtype=np.float64)

    aligned_timestamps = []
    tcp_pose = []
    tcp_velocity = []
    tcp_wrench = []
    joint_positions = []
    joint_velocities = []
    gripper_width = []
    gripper_force = []
    controller_valid = []
    controller_age_sec = []
    controller_source_timestamps = []
    teleop_target = []
    teleop_closed = []
    teleop_source_timestamps = []
    teleop_action_lead_sec = []
    marker_locations = []
    marker_offsets = []
    gelsight_capture_times = []
    orbbec_frame_paths = []
    orbbec_capture_times = []
    dropped_without_future_action = 0
    dropped_action_outside_horizon = 0

    for timestamp in grid:
        controller_item, controller_timestamp = _latest_record(controller, controller_times, timestamp)
        if controller_item is None:
            continue
        teleop_item, teleop_timestamp = _next_record(teleop, teleop_times, timestamp)
        if teleop_item is None:
            dropped_without_future_action += 1
            continue
        action_lead_sec = teleop_timestamp - timestamp
        if action_lead_sec <= 0.0:
            dropped_without_future_action += 1
            continue
        if action_lead_sec > action_horizon_sec:
            dropped_action_outside_horizon += 1
            continue

        controller_state = controller_item["state"]
        aligned_timestamps.append(timestamp)
        tcp_pose.append(controller_state["tcp_pose"])
        tcp_velocity.append(controller_state["tcp_velocity"])
        tcp_wrench.append(controller_state["tcp_wrench"])
        joint_positions.append(controller_state.get("joint_positions", [0.0] * 7))
        joint_velocities.append(controller_state.get("joint_velocities", [0.0] * 7))
        gripper_width.append(float(controller_state["gripper_width"]))
        gripper_force.append(float(controller_state["gripper_force"]))
        controller_age_sec.append(timestamp - controller_timestamp)
        controller_source_timestamps.append(controller_timestamp)
        controller_valid.append(True)
        teleop_target.append(teleop_item["target_tcp"])
        teleop_closed.append(bool(teleop_item["gripper_closed"]))
        teleop_source_timestamps.append(teleop_timestamp)
        teleop_action_lead_sec.append(action_lead_sec)

        gelsight_item, gelsight_timestamp = _latest_record(gelsight, gelsight_times, timestamp)
        if gelsight_item is None:
            marker_locations.append([])
            marker_offsets.append([])
            gelsight_capture_times.append(np.nan)
        else:
            marker_locations.append(gelsight_item["marker_locations"])
            marker_offsets.append(gelsight_item["marker_offsets"])
            gelsight_capture_times.append(gelsight_timestamp)

        orbbec_item, orbbec_timestamp = _latest_record(orbbec, orbbec_times, timestamp)
        if orbbec_item is None:
            orbbec_frame_paths.append("")
            orbbec_capture_times.append(np.nan)
        else:
            orbbec_frame_paths.append(orbbec_item.get("frame_path", ""))
            orbbec_capture_times.append(orbbec_timestamp)

    output_path = episode_dir / "aligned_episode.npz"
    np.savez_compressed(
        output_path,
        timestamps=np.asarray(aligned_timestamps, dtype=np.float64),
        robot_tcp_pose=_float_matrix(tcp_pose, width=7),
        robot_tcp_velocity=_float_matrix(tcp_velocity, width=6),
        robot_tcp_wrench=_float_matrix(tcp_wrench, width=6),
        robot_joint_positions=_float_matrix(joint_positions, width=7),
        robot_joint_velocities=_float_matrix(joint_velocities, width=7),
        gripper_width=np.asarray(gripper_width, dtype=np.float64),
        gripper_force=np.asarray(gripper_force, dtype=np.float64),
        gripper_state=np.asarray(list(zip(gripper_width, gripper_force)), dtype=np.float64),
        controller_state_valid=np.asarray(controller_valid, dtype=bool),
        controller_state_age_sec=np.asarray(controller_age_sec, dtype=np.float64),
        controller_state_source_timestamps=np.asarray(controller_source_timestamps, dtype=np.float64),
        teleop_target_tcp=_float_matrix(teleop_target, width=7),
        teleop_gripper_closed=np.asarray(teleop_closed, dtype=bool),
        teleop_command_source_timestamps=np.asarray(teleop_source_timestamps, dtype=np.float64),
        teleop_action_lead_sec=np.asarray(teleop_action_lead_sec, dtype=np.float64),
        gelsight_marker_locations=np.asarray(marker_locations, dtype=object),
        gelsight_marker_offsets=np.asarray(marker_offsets, dtype=object),
        gelsight_capture_timestamps=np.asarray(gelsight_capture_times, dtype=np.float64),
        orbbec_rgb_frame_paths=np.asarray(orbbec_frame_paths, dtype=object),
        orbbec_rgb_capture_timestamps=np.asarray(orbbec_capture_times, dtype=np.float64),
    )
    streams_used = ["controller_state"]
    if teleop:
        streams_used.append("teleop_commands")
    if gelsight:
        streams_used.append("gelsight_markers")
    if orbbec:
        streams_used.append("orbbec_rgb")
    manifest = {
        "target_hz": target_hz,
        "num_steps": int(len(aligned_timestamps)),
        "grid_steps": int(len(grid)),
        "alignment_mode": "causal_observation_future_action",
        "action_horizon_sec": action_horizon_sec,
        "dropped_steps_without_future_action": int(dropped_without_future_action),
        "dropped_steps_action_outside_horizon": int(dropped_action_outside_horizon),
        "streams_used": streams_used,
    }
    (episode_dir / "aligned_episode_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return output_path
