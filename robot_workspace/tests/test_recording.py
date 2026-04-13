from pathlib import Path

from vt_franka_workspace.recording.postprocess import align_episode
from vt_franka_workspace.recording.raw_recorder import JsonlStreamRecorder
from vt_franka_workspace.recording.session import EpisodeSessionManager


def test_episode_session_and_alignment(tmp_path: Path):
    sessions = EpisodeSessionManager(tmp_path)
    episode_dir = sessions.start_episode("test")

    controller = JsonlStreamRecorder(sessions, "controller_state")
    teleop = JsonlStreamRecorder(sessions, "teleop_commands")
    gelsight = JsonlStreamRecorder(sessions, "gelsight_markers")

    controller.record_event(
        {
            "source_wall_time": 1.0,
            "state": {
                "tcp_pose": [0.1] * 7,
                "tcp_velocity": [0.0] * 6,
                "tcp_wrench": [0.0] * 6,
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
                "gripper_width": 0.01,
                "gripper_force": 5.0,
            },
        }
    )
    teleop.record_event({"source_wall_time": 1.0, "target_tcp": [0.3] * 7, "gripper_closed": True})
    gelsight.record_event({"captured_wall_time": 1.0, "marker_locations": [[0.1, 0.2]], "marker_offsets": [[0.0, 0.1]]})
    sessions.stop_episode()

    output_path = align_episode(episode_dir, target_hz=10.0)
    assert output_path.exists()
