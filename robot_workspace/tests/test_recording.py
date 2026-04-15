from pathlib import Path

import json

import numpy as np

from vt_franka_workspace.recording.postprocess import align_episode
from vt_franka_workspace.recording.raw_recorder import JsonlStreamRecorder
from vt_franka_workspace.recording.qc import analyze_episode
from vt_franka_workspace.recording.session import RunSessionManager


def test_run_session_and_alignment(tmp_path: Path):
    sessions = RunSessionManager(tmp_path / "runs")
    sessions.start_run("test")
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
                "gripper_width": 0.01,
                "gripper_force": 5.0,
            },
        }
    )
    teleop.record_event({"source_wall_time": 1.0, "target_tcp": [0.3] * 7, "gripper_closed": True})
    teleop.record_event({"source_wall_time": 1.05, "target_tcp": [0.4] * 7, "gripper_closed": False})
    teleop.record_event({"source_wall_time": 1.15, "target_tcp": [0.5] * 7, "gripper_closed": True})
    gelsight.record_event({"captured_wall_time": 1.0, "marker_locations": [[0.1, 0.2]], "marker_offsets": [[0.0, 0.1]]})
    sessions.stop_episode()

    output_path = align_episode(episode_dir, target_hz=10.0)
    assert output_path.exists()
    aligned = np.load(output_path, allow_pickle=True)
    assert "robot_joint_positions" in aligned
    assert aligned["robot_joint_positions"].shape[1] == 7
    assert aligned["timestamps"].tolist() == [1.0, 1.1]
    assert aligned["teleop_command_source_timestamps"].tolist() == [1.05, 1.15]
    assert np.all(aligned["teleop_action_lead_sec"] > 0.0)


def test_run_session_manager_creates_nested_run_and_qc(tmp_path: Path):
    sessions = RunSessionManager(tmp_path / "runs")
    run_dir = sessions.start_run("task_demo", metadata={"operator": "tester"})
    episode_dir = sessions.start_episode()

    controller = JsonlStreamRecorder(sessions, "controller_state")
    teleop = JsonlStreamRecorder(sessions, "teleop_commands")
    controller.record_event(
        {
            "source_wall_time": 1.0,
            "state": {
                "tcp_pose": [0.1] * 7,
                "tcp_velocity": [0.0] * 6,
                "tcp_wrench": [0.0] * 6,
                "joint_positions": [0.1] * 7,
                "joint_velocities": [0.0] * 7,
                "gripper_width": 0.05,
                "gripper_force": 0.0,
            },
        },
        event_time=1.0,
    )
    teleop.record_event({"source_wall_time": 1.0, "target_tcp": [0.1] * 7, "gripper_closed": False}, event_time=1.0)
    sessions.stop_episode(outcome="saved")
    qc_path = analyze_episode(episode_dir)
    sessions.stop_run()

    run_manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))
    assert run_manifest["run_name"] == "task_demo"
    assert episode_dir.parent.name == "episodes"
    assert qc_path.exists()


def test_run_session_manager_resumes_and_increments_episode_index(tmp_path: Path):
    sessions = RunSessionManager(tmp_path / "runs")
    run_dir = sessions.start_run("task_demo")
    episode_dir = sessions.start_episode()
    sessions.stop_episode(outcome="saved")
    sessions.stop_run()

    resumed = RunSessionManager(tmp_path / "runs")
    resumed_run_dir = resumed.start_run("task_demo")
    next_episode_dir = resumed.start_episode()

    assert resumed_run_dir == run_dir
    assert episode_dir.name == "episode_0000"
    assert next_episode_dir.name == "episode_0001"


def test_jsonl_stream_recorder_respects_record_hz(tmp_path: Path):
    sessions = RunSessionManager(tmp_path / "runs")
    sessions.start_run("rate_limited")
    sessions.start_episode("rate_limited")
    recorder = JsonlStreamRecorder(sessions, "controller_state", record_hz=2.0)

    recorder.record_event({"source_wall_time": 1.0, "state": {"tcp_pose": [0.0] * 7}}, event_time=1.0)
    recorder.record_event({"source_wall_time": 1.1, "state": {"tcp_pose": [0.0] * 7}}, event_time=1.1)
    recorder.record_event({"source_wall_time": 1.6, "state": {"tcp_pose": [0.0] * 7}}, event_time=1.6)

    path = sessions.get_active_episode_dir() / "streams" / "controller_state.jsonl"
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 2
