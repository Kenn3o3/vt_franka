from __future__ import annotations

import numbers
from typing import Union

import numpy as np
import scipy.interpolate as si
import scipy.spatial.transform as st


def rotation_distance(a: st.Rotation, b: st.Rotation) -> float:
    return (b * a.inv()).magnitude()


def pose_distance(start_pose, end_pose):
    start_pose = np.asarray(start_pose, dtype=np.float64)
    end_pose = np.asarray(end_pose, dtype=np.float64)
    start_pos = start_pose[:3]
    end_pos = end_pose[:3]
    start_rot = st.Rotation.from_rotvec(start_pose[3:])
    end_rot = st.Rotation.from_rotvec(end_pose[3:])
    return np.linalg.norm(end_pos - start_pos), rotation_distance(start_rot, end_rot)


class PoseTrajectoryInterpolator:
    def __init__(self, times: np.ndarray, poses: np.ndarray):
        if not isinstance(times, np.ndarray):
            times = np.asarray(times, dtype=np.float64)
        if not isinstance(poses, np.ndarray):
            poses = np.asarray(poses, dtype=np.float64)
        if len(times) != len(poses):
            raise ValueError("times and poses must have the same length")
        if len(times) < 1:
            raise ValueError("at least one waypoint is required")
        if poses.shape[1] != 6:
            raise ValueError("poses must have shape (N, 6)")

        if len(times) == 1:
            self.single_step = True
            self._times = times
            self._poses = poses
        else:
            self.single_step = False
            if not np.all(times[1:] >= times[:-1]):
                raise ValueError("times must be monotonically increasing")
            pos = poses[:, :3]
            rot = st.Rotation.from_rotvec(poses[:, 3:])
            self.pos_interp = si.interp1d(times, pos, axis=0, assume_sorted=True)
            self.rot_interp = st.Slerp(times, rot)

    @property
    def times(self) -> np.ndarray:
        return self._times if self.single_step else self.pos_interp.x

    @property
    def poses(self) -> np.ndarray:
        if self.single_step:
            return self._poses
        poses = np.zeros((len(self.times), 6), dtype=np.float64)
        poses[:, :3] = self.pos_interp.y
        poses[:, 3:] = self.rot_interp(self.times).as_rotvec()
        return poses

    def trim(self, start_t: float, end_t: float) -> "PoseTrajectoryInterpolator":
        if start_t > end_t:
            raise ValueError("start_t must be <= end_t")
        times = self.times
        keep_mask = (start_t < times) & (times < end_t)
        keep_times = times[keep_mask]
        all_times = np.unique(np.concatenate([[start_t], keep_times, [end_t]]))
        return PoseTrajectoryInterpolator(all_times, self(all_times))

    def schedule_waypoint(
        self,
        pose,
        time,
        max_pos_speed=np.inf,
        max_rot_speed=np.inf,
        curr_time=None,
        last_waypoint_time=None,
    ) -> "PoseTrajectoryInterpolator":
        if max_pos_speed <= 0 or max_rot_speed <= 0:
            raise ValueError("speed limits must be positive")

        start_time = self.times[0]
        end_time = self.times[-1]
        if curr_time is not None:
            if time <= curr_time:
                return self
            start_time = max(curr_time, start_time)
            if last_waypoint_time is not None:
                end_time = curr_time if time <= last_waypoint_time else max(last_waypoint_time, curr_time)
            else:
                end_time = curr_time

        end_time = min(end_time, time)
        start_time = min(start_time, end_time)
        trimmed_interp = self.trim(start_time, end_time)

        duration = time - end_time
        end_pose = trimmed_interp(end_time)
        pos_dist, rot_dist = pose_distance(pose, end_pose)
        duration = max(duration, pos_dist / max_pos_speed, rot_dist / max_rot_speed)
        last_waypoint_time = end_time + duration

        times = np.append(trimmed_interp.times, [last_waypoint_time], axis=0)
        poses = np.append(trimmed_interp.poses, [pose], axis=0)
        return PoseTrajectoryInterpolator(times, poses)

    def __call__(self, t: Union[numbers.Number, np.ndarray]) -> np.ndarray:
        single = isinstance(t, numbers.Number)
        if single:
            t = np.asarray([t], dtype=np.float64)
        else:
            t = np.asarray(t, dtype=np.float64)

        if self.single_step:
            pose = np.repeat(self._poses[[0]], len(t), axis=0)
        else:
            t = np.clip(t, self.times[0], self.times[-1])
            pose = np.zeros((len(t), 6), dtype=np.float64)
            pose[:, :3] = self.pos_interp(t)
            pose[:, 3:] = self.rot_interp(t).as_rotvec()

        return pose[0] if single else pose

