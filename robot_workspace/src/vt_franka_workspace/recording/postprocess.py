from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

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


def _nearest_record(records: list[dict[str, Any]], timestamp: float, timestamp_key: str) -> dict[str, Any] | None:
    if not records:
        return None
    best = min(records, key=lambda item: abs(float(item[timestamp_key]) - timestamp))
    return best


def align_episode(episode_dir: str | Path, target_hz: float = 24.0) -> Path:
    episode_dir = Path(episode_dir)
    streams_dir = episode_dir / "streams"
    controller = _read_jsonl(streams_dir / "controller_state.jsonl")
    teleop = _read_jsonl(streams_dir / "teleop_commands.jsonl")
    gelsight = _read_jsonl(streams_dir / "gelsight_markers.jsonl")
    orbbec = _read_jsonl(streams_dir / "orbbec_rgb.jsonl")

    if not controller:
        raise RuntimeError(f"No controller_state stream found in {episode_dir}")

    controller_times = np.array([float(item["source_wall_time"]) for item in controller], dtype=np.float64)
    start_time = controller_times[0]
    end_time = controller_times[-1]
    step = 1.0 / target_hz
    grid = np.arange(start_time, end_time + step * 0.5, step, dtype=np.float64)

    tcp_pose = []
    tcp_velocity = []
    tcp_wrench = []
    joint_positions = []
    joint_velocities = []
    gripper_width = []
    gripper_force = []
    controller_valid = []
    controller_age_sec = []
    teleop_target = []
    teleop_closed = []
    marker_locations = []
    marker_offsets = []
    orbbec_frame_paths = []
    orbbec_capture_times = []

    for timestamp in grid:
        controller_item = _nearest_record(controller, timestamp, "source_wall_time")
        if controller_item is None:
            continue
        controller_state = controller_item["state"]
        controller_timestamp = float(controller_item["source_wall_time"])
        tcp_pose.append(controller_item["state"]["tcp_pose"])
        tcp_velocity.append(controller_item["state"]["tcp_velocity"])
        tcp_wrench.append(controller_item["state"]["tcp_wrench"])
        joint_positions.append(controller_state.get("joint_positions", [0.0] * 7))
        joint_velocities.append(controller_state.get("joint_velocities", [0.0] * 7))
        gripper_width.append(float(controller_state["gripper_width"]))
        gripper_force.append(float(controller_state["gripper_force"]))
        controller_age_sec.append(abs(controller_timestamp - timestamp))
        controller_valid.append(True)

        teleop_item = _nearest_record(teleop, timestamp, "source_wall_time")
        if teleop_item is None:
            teleop_target.append([0.0] * 7)
            teleop_closed.append(False)
        else:
            teleop_target.append(teleop_item["target_tcp"])
            teleop_closed.append(bool(teleop_item["gripper_closed"]))

        gelsight_item = _nearest_record(gelsight, timestamp, "captured_wall_time")
        if gelsight_item is None:
            marker_locations.append([])
            marker_offsets.append([])
        else:
            marker_locations.append(gelsight_item["marker_locations"])
            marker_offsets.append(gelsight_item["marker_offsets"])

        orbbec_item = _nearest_record(orbbec, timestamp, "captured_wall_time")
        if orbbec_item is None:
            orbbec_frame_paths.append("")
            orbbec_capture_times.append(np.nan)
        else:
            orbbec_frame_paths.append(orbbec_item.get("frame_path", ""))
            orbbec_capture_times.append(float(orbbec_item["captured_wall_time"]))

    output_path = episode_dir / "aligned_episode.npz"
    np.savez_compressed(
        output_path,
        timestamps=grid,
        robot_tcp_pose=np.asarray(tcp_pose, dtype=object),
        robot_tcp_velocity=np.asarray(tcp_velocity, dtype=object),
        robot_tcp_wrench=np.asarray(tcp_wrench, dtype=object),
        robot_joint_positions=np.asarray(joint_positions, dtype=np.float64),
        robot_joint_velocities=np.asarray(joint_velocities, dtype=np.float64),
        gripper_width=np.asarray(gripper_width, dtype=np.float64),
        gripper_force=np.asarray(gripper_force, dtype=np.float64),
        gripper_state=np.asarray(list(zip(gripper_width, gripper_force)), dtype=np.float64),
        controller_state_valid=np.asarray(controller_valid, dtype=bool),
        controller_state_age_sec=np.asarray(controller_age_sec, dtype=np.float64),
        teleop_target_tcp=np.asarray(teleop_target, dtype=object),
        teleop_gripper_closed=np.asarray(teleop_closed, dtype=bool),
        gelsight_marker_locations=np.asarray(marker_locations, dtype=object),
        gelsight_marker_offsets=np.asarray(marker_offsets, dtype=object),
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
        "num_steps": int(len(grid)),
        "streams_used": streams_used,
    }
    (episode_dir / "aligned_episode_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return output_path
