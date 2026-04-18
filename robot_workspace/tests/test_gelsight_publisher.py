from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("cv2")

from vt_franka_workspace.sensors.gelsight import publisher


def test_filter_candidates_matches_name_and_serial():
    candidates = [
        {"name": "Orbbec Gemini", "serial_number": "AAA", "device_path": "/dev/video0"},
        {"name": "GelSight Mini R0B", "serial_number": "2BUUCE3E", "device_path": "/dev/video12"},
    ]

    matched = publisher._filter_candidates(
        candidates,
        name_contains="GelSight Mini",
        serial_number="2BUUCE3E",
    )

    assert matched == [{"name": "GelSight Mini R0B", "serial_number": "2BUUCE3E", "device_path": "/dev/video12"}]


def test_query_v4l_device_detects_metadata_only_node(monkeypatch, tmp_path: Path):
    device = tmp_path / "video13"
    device.write_text("", encoding="utf-8")

    class FakeCompletedProcess:
        def __init__(self, stdout: str):
            self.returncode = 0
            self.stdout = stdout

    stdout = "\n".join(
        [
            "Card type        : GelSight Mini R0B 2BUU-CE3E: Ge",
            "Serial           : 2BUUCE3E",
            "Device Caps      : 0x04a00000",
            "\tMetadata Capture",
            "\tStreaming",
        ]
    )
    monkeypatch.setattr(
        publisher.subprocess,
        "run",
        lambda *args, **kwargs: FakeCompletedProcess(stdout),
    )

    info = publisher._query_v4l_device(device)

    assert info is not None
    assert info["video_capture"] is False
    assert info["serial_number"] == "2BUUCE3E"
