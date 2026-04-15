from pathlib import Path

import numpy as np

from vt_franka_workspace.recording.postprocess import align_episode
from vt_franka_workspace.recording.raw_recorder import JsonlStreamRecorder
from vt_franka_workspace.recording.session import RunSessionManager


def test_align_episode_drops_steps_without_future_action(tmp_path: Path):
    sessions = RunSessionManager(tmp_path / "runs")
    sessions.start_run("alignment")
    episode_dir = sessions.start_episode("alignment")

    controller = JsonlStreamRecorder(sessions, "controller_state")
    teleop = JsonlStreamRecorder(sessions, "teleop_commands")

    for source_wall_time in (1.0, 1.1, 1.2, 1.3):
        controller.record_event(
            {
                "source_wall_time": source_wall_time,
                "state": {
                    "tcp_pose": [source_wall_time] * 7,
                    "tcp_velocity": [0.0] * 6,
                    "tcp_wrench": [0.0] * 6,
                    "joint_positions": [source_wall_time] * 7,
                    "joint_velocities": [0.0] * 7,
                    "gripper_width": 0.07,
                    "gripper_force": 0.0,
                },
            }
        )

    teleop.record_event({"source_wall_time": 1.05, "target_tcp": [0.1] * 7, "gripper_closed": False})
    teleop.record_event({"source_wall_time": 1.15, "target_tcp": [0.2] * 7, "gripper_closed": True})
    sessions.stop_episode()

    output_path = align_episode(episode_dir, target_hz=10.0)
    aligned = np.load(output_path, allow_pickle=True)

    assert output_path.exists()
    assert aligned["timestamps"].tolist() == [1.0, 1.1]
    assert aligned["teleop_command_source_timestamps"].tolist() == [1.05, 1.15]
    assert aligned["robot_tcp_pose"].shape == (2, 7)
    assert np.all(aligned["controller_state_source_timestamps"] <= aligned["timestamps"])
    assert np.all(aligned["teleop_command_source_timestamps"] > aligned["timestamps"])

