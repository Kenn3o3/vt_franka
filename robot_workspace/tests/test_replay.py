from pathlib import Path

import numpy as np

from vt_franka_workspace.rollout.replay_policy import build_replay_policy, infer_sync_hz, load_replay_episode
from vt_franka_workspace.settings import TeleopSettings


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


def test_build_replay_policy_returns_sequenced_actions(tmp_path: Path):
    path = tmp_path / "aligned_episode.npz"
    np.savez_compressed(
        path,
        timestamps=np.array([10.0, 10.1, 10.2], dtype=np.float64),
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

    policy = build_replay_policy(
        episode_dir=path,
        hz=10.0,
        speed_scale=1.0,
        skip_gripper=False,
        teleop_settings=TeleopSettings(),
    )

    assert policy.__vt_franka_control_hz__ == 10.0
    action0 = policy({})
    action1 = policy({})
    assert action0["target_tcp"] == [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0]
    assert "gripper_width" in action0
    assert action1["target_tcp"] == [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0]


def test_build_replay_policy_requires_episode_dir():
    try:
        build_replay_policy(teleop_settings=TeleopSettings())
    except ValueError as exc:
        assert "episode-dir" in str(exc)
    else:
        raise AssertionError("Expected ValueError when episode_dir is missing")
