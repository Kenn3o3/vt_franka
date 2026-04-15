from __future__ import annotations

import threading
import time
from typing import Sequence

import numpy as np

from vt_franka_shared.models import ControllerState
from vt_franka_shared.pose_math import pose6d_to_pose7d, pose7d_to_pose6d

from .base import FrankaBackend


class MockFrankaBackend(FrankaBackend):
    name = "mock"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose6d = np.zeros(6, dtype=np.float64)
        self._gripper_width = 0.078
        self._gripper_force = 0.0
        self._impedance_started = False

    def get_tcp_pose(self) -> np.ndarray:
        with self._lock:
            return pose6d_to_pose7d(self._pose6d)

    def get_controller_state(self, control_frequency_hz: float) -> ControllerState:
        with self._lock:
            pose6d = self._pose6d.copy()
            gripper_width = self._gripper_width
            gripper_force = self._gripper_force
        return ControllerState(
            tcp_pose=pose6d_to_pose7d(pose6d).tolist(),
            tcp_velocity=[0.0] * 6,
            tcp_wrench=[0.0] * 6,
            joint_positions=[0.0] * 7,
            joint_velocities=[0.0] * 7,
            gripper_width=gripper_width,
            gripper_force=gripper_force,
            control_frequency_hz=control_frequency_hz,
            backend=self.name,
        )

    def start_cartesian_impedance(self, stiffness: Sequence[float], damping: Sequence[float]) -> None:
        self._impedance_started = True

    def update_desired_tcp(self, target_pose6d: np.ndarray) -> None:
        with self._lock:
            self._pose6d = np.asarray(target_pose6d, dtype=np.float64)

    def move_gripper(self, width: float, velocity: float, force_limit: float) -> None:
        with self._lock:
            self._gripper_width = width
            self._gripper_force = 0.0

    def grasp(self, velocity: float, force_limit: float) -> None:
        with self._lock:
            self._gripper_width = 0.0
            self._gripper_force = force_limit

    def stop_gripper(self) -> None:
        return None

    def go_home(self, ee_pose: Sequence[float], duration_sec: float) -> None:
        with self._lock:
            ee_pose = np.asarray(ee_pose, dtype=np.float64)
            self._pose6d = np.concatenate([ee_pose[:3], np.deg2rad(ee_pose[3:])])
        time.sleep(min(duration_sec, 0.01))

    def shutdown(self) -> None:
        self._impedance_started = False
