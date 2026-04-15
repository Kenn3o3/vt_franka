from pathlib import Path

import numpy as np
import pytest

from vt_franka_workspace.recording.postprocess import align_episode
from vt_franka_workspace.recording.raw_recorder import JsonlStreamRecorder
from vt_franka_workspace.recording.session import RunSessionManager
from vt_franka_workspace.sensors.orbbec.frame_decoder import decode_color_buffer


def test_decode_rgb_buffer_to_bgr():
    pytest.importorskip("cv2")
    rgb = np.array([[[255, 0, 0], [0, 255, 0]]], dtype=np.uint8)
    image = decode_color_buffer(rgb.tobytes(), width=2, height=1, color_format="RGB")
    assert image.tolist() == [[[0, 0, 255], [0, 255, 0]]]


def test_align_episode_includes_orbbec_stream(tmp_path: Path):
    sessions = RunSessionManager(tmp_path / "runs")
    sessions.start_run("orbbec")
    episode_dir = sessions.start_episode("orbbec")

    controller = JsonlStreamRecorder(sessions, "controller_state")
    orbbec = JsonlStreamRecorder(sessions, "orbbec_rgb")

    controller.record_event(
        {
            "source_wall_time": 1.0,
            "state": {
                "tcp_pose": [0.1] * 7,
                "tcp_velocity": [0.0] * 6,
                "tcp_wrench": [0.0] * 6,
                "joint_positions": [0.1] * 7,
                "joint_velocities": [0.0] * 7,
                "gripper_width": 0.07,
                "gripper_force": 0.0,
            },
        }
    )
    controller.record_event(
        {
            "source_wall_time": 1.1,
            "state": {
                "tcp_pose": [0.2] * 7,
                "tcp_velocity": [0.0] * 6,
                "tcp_wrench": [0.0] * 6,
                "joint_positions": [0.2] * 7,
                "joint_velocities": [0.0] * 7,
                "gripper_width": 0.02,
                "gripper_force": 3.0,
            },
        }
    )
    orbbec.record_event(
        {
            "captured_wall_time": 1.0,
            "frame_path": "streams/orbbec_rgb/000001.jpg",
            "frame_width": 640,
            "frame_height": 480,
        }
    )
    sessions.stop_episode()

    output_path = align_episode(episode_dir, target_hz=10.0)
    aligned = np.load(output_path, allow_pickle=True)
    assert output_path.exists()
    assert "orbbec_rgb_frame_paths" in aligned
    assert aligned["orbbec_rgb_frame_paths"][0] == "streams/orbbec_rgb/000001.jpg"
