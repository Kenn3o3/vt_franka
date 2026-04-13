import numpy as np

from vt_franka_shared.interpolation import PoseTrajectoryInterpolator


def test_schedule_waypoint_advances_pose():
    interp = PoseTrajectoryInterpolator(
        times=np.array([0.0]),
        poses=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]),
    )
    interp = interp.schedule_waypoint(
        pose=np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
        time=1.0,
        curr_time=0.1,
        last_waypoint_time=0.0,
    )
    pose = interp(1.0)
    assert np.allclose(pose[:3], [0.2, 0.0, 0.0])

