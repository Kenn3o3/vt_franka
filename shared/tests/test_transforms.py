from pathlib import Path

import numpy as np

from vt_franka_shared.transforms import SingleArmCalibration


def test_calibration_loads_v6_assets():
    repo_root = Path(__file__).resolve().parents[2]
    calibration = SingleArmCalibration.from_dir(repo_root / "robot_workspace/config/calibration/v6")
    assert calibration.world_to_robot_base.shape == (4, 4)
    unity_pose = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    robot_pose = calibration.unity_to_robot_pose(unity_pose)
    assert robot_pose.shape == (7,)
