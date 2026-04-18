from pathlib import Path

import json
import numpy as np

from vt_franka_workspace.visualization import export_episode_composite_video


def test_export_episode_composite_video_creates_mp4(tmp_path: Path):
    episode_dir = tmp_path / "episode_0000"
    streams_dir = episode_dir / "streams" / "rgb_third_person"
    streams_dir.mkdir(parents=True)

    try:
        import cv2
    except ImportError:
        return

    frames = []
    for idx, value in enumerate((40, 120, 220)):
        frame = np.full((48, 64, 3), value, dtype=np.uint8)
        frame_path = streams_dir / f"frame{idx}.jpg"
        assert cv2.imwrite(str(frame_path), frame)
        frames.append(f"streams/rgb_third_person/frame{idx}.jpg")

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
        rgb_third_person_frame_paths=np.array(frames, dtype=object),
        rgb_third_person_capture_timestamps=timestamps,
    )
    (episode_dir / "episode_manifest.json").write_text(
        json.dumps(
            {
                "metadata": {
                    "instruction": "Move the bowl to the right 10 cm.",
                    "start_state": {"x": 0.03, "y": 0.4, "z": 0.14, "yaw_deg": 45},
                    "goal_state": {"x": 0.13, "y": 0.4, "z": 0.14, "yaw_deg": 45},
                }
            }
        ),
        encoding="utf-8",
    )

    output = export_episode_composite_video(episode_dir)
    assert output.exists()
    assert output.suffix == ".mp4"
