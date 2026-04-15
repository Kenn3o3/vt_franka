from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


def analyze_episode(
    episode_dir: str | Path,
    controller_gap_warn_sec: float = 0.5,
    controller_gap_fail_sec: float = 2.0,
) -> Path:
    episode_dir = Path(episode_dir)
    streams_dir = episode_dir / "streams"

    controller = _read_jsonl(streams_dir / "controller_state.jsonl")
    teleop = _read_jsonl(streams_dir / "teleop_commands.jsonl")
    quest = _read_jsonl(streams_dir / "quest_messages.jsonl")
    orbbec = _read_jsonl(streams_dir / "orbbec_rgb.jsonl")
    gelsight = _read_jsonl(streams_dir / "gelsight_markers.jsonl")

    controller_summary = _summarize_stream(controller, "source_wall_time")
    teleop_summary = _summarize_stream(teleop, "source_wall_time")
    quest_summary = _summarize_stream(quest, "source_wall_time")
    orbbec_summary = _summarize_stream(orbbec, "captured_wall_time")
    gelsight_summary = _summarize_stream(gelsight, "captured_wall_time")

    usable_for_training = bool(controller) and controller_summary["max_gap_sec"] < controller_gap_fail_sec
    status = "pass" if usable_for_training else "fail"
    if usable_for_training and controller_summary["max_gap_sec"] >= controller_gap_warn_sec:
        status = "warn"

    qc = {
        "episode_dir": str(episode_dir),
        "generated_at_wall_time": time.time(),
        "status": status,
        "usable_for_training": usable_for_training,
        "thresholds": {
            "controller_gap_warn_sec": controller_gap_warn_sec,
            "controller_gap_fail_sec": controller_gap_fail_sec,
        },
        "streams": {
            "controller_state": controller_summary,
            "teleop_commands": teleop_summary,
            "quest_messages": quest_summary,
            "orbbec_rgb": orbbec_summary,
            "gelsight_markers": gelsight_summary,
        },
    }

    output_path = episode_dir / "qc.json"
    output_path.write_text(json.dumps(qc, indent=2), encoding="utf-8")
    return output_path


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


def _summarize_stream(records: list[dict[str, Any]], timestamp_key: str) -> dict[str, Any]:
    if not records:
        return {"count": 0, "duration_sec": 0.0, "effective_hz": 0.0, "max_gap_sec": 0.0}

    timestamps = [float(record[timestamp_key]) for record in records]
    start_time = timestamps[0]
    end_time = timestamps[-1]
    duration_sec = max(0.0, end_time - start_time)
    if len(timestamps) > 1:
        gaps = [curr - prev for prev, curr in zip(timestamps, timestamps[1:])]
        max_gap_sec = max(gaps)
    else:
        max_gap_sec = 0.0

    effective_hz = len(records) / duration_sec if duration_sec > 1e-6 else 0.0
    return {
        "count": len(records),
        "start_time": start_time,
        "end_time": end_time,
        "duration_sec": duration_sec,
        "effective_hz": effective_hz,
        "max_gap_sec": max_gap_sec,
    }
