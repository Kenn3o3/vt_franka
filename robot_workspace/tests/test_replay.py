from pathlib import Path

import numpy as np

from vt_franka_workspace.replay import infer_sync_hz, load_replay_episode, replay_episode
from vt_franka_workspace.settings import TeleopSettings


class FakeController:
    def __init__(self) -> None:
        self.tcp_commands = []
        self.gripper_commands = []

    def queue_tcp(self, target_tcp, source="test"):
        self.tcp_commands.append((target_tcp, source))

    def move_gripper(self, width, velocity, force_limit, source="test"):
        self.gripper_commands.append(("move", width, velocity, force_limit, source))

    def grasp_gripper(self, velocity, force_limit, source="test"):
        self.gripper_commands.append(("grasp", velocity, force_limit, source))


def test_infer_sync_hz_from_timestamps():
    timestamps = np.array([0.0, 0.1, 0.2, 0.3], dtype=np.float64)
    assert infer_sync_hz(timestamps) == 10.0


def test_load_replay_episode_reads_aligned_npz(tmp_path: Path):
    path = tmp_path / "aligned_episode.npz"
    np.savez_compressed(
        path,
        timestamps=np.array([0.0, 0.1, 0.2], dtype=np.float64),
        teleop_target_tcp=np.array(
            [
                [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
                [0.2, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
                [0.3, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        ),
        teleop_gripper_closed=np.array([False, True, True], dtype=bool),
    )

    episode = load_replay_episode(path)
    assert episode.hz == 10.0
    assert episode.target_tcp.shape == (3, 7)
    assert episode.gripper_closed.tolist() == [False, True, True]


def test_replay_episode_sends_tcp_and_gripper_commands():
    controller = FakeController()
    episode = load_replay_episode_from_arrays()
    replay_episode(
        controller,
        episode,
        teleop_settings=TeleopSettings(),
        hz=1000.0,
    )

    assert len(controller.tcp_commands) == 3
    assert controller.gripper_commands[0][0] == "move"
    assert controller.gripper_commands[1][0] == "grasp"


def load_replay_episode_from_arrays():
    class Episode:
        timestamps = np.array([0.0, 0.1, 0.2], dtype=np.float64)
        target_tcp = np.array(
            [
                [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
                [0.2, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
                [0.3, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
        gripper_closed = np.array([False, True, True], dtype=bool)
        hz = 10.0

    return Episode()
