from pathlib import Path

import numpy as np

from vt_franka_workspace.visualization import visualize_aligned_episode


def test_visualize_aligned_episode_creates_output(tmp_path: Path):
    episode_dir = tmp_path / "episode_0000"
    streams_dir = episode_dir / "streams" / "orbbec_rgb"
    streams_dir.mkdir(parents=True)

    try:
        import cv2
    except ImportError:
        return

    frame = np.full((48, 64, 3), 180, dtype=np.uint8)
    frame_path = streams_dir / "frame0.jpg"
    assert cv2.imwrite(str(frame_path), frame)

    timestamps = np.array([1.0, 1.1, 1.2], dtype=np.float64)
    pose = np.array(
        [
            [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
            [0.2, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
            [0.3, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    np.savez_compressed(
        episode_dir / "aligned_episode.npz",
        timestamps=timestamps,
        robot_tcp_pose=pose,
        robot_tcp_velocity=np.zeros((3, 6), dtype=np.float64),
        robot_tcp_wrench=np.zeros((3, 6), dtype=np.float64),
        robot_joint_positions=np.zeros((3, 7), dtype=np.float64),
        robot_joint_velocities=np.zeros((3, 7), dtype=np.float64),
        gripper_width=np.zeros((3,), dtype=np.float64),
        gripper_force=np.zeros((3,), dtype=np.float64),
        gripper_state=np.zeros((3, 2), dtype=np.float64),
        controller_state_valid=np.ones((3,), dtype=bool),
        controller_state_age_sec=np.zeros((3,), dtype=np.float64),
        teleop_target_tcp=pose,
        teleop_gripper_closed=np.zeros((3,), dtype=bool),
        gelsight_marker_locations=np.empty((3, 0), dtype=object),
        gelsight_marker_offsets=np.empty((3, 0), dtype=object),
        orbbec_rgb_frame_paths=np.array(["streams/orbbec_rgb/frame0.jpg"] * 3, dtype=object),
        orbbec_rgb_capture_timestamps=timestamps,
    )

    output = visualize_aligned_episode(episode_dir)
    assert output.exists()
