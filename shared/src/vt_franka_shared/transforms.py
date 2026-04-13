from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Union

import numpy as np

from .pose_math import pose7d_to_matrix


def _load_matrix(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as handle:
        return np.asarray(json.load(handle), dtype=np.float64)


@dataclass
class SingleArmCalibration:
    calibration_dir: Path
    external_camera_to_robot_base: np.ndarray = field(init=False)
    left_wrist_camera_to_tcp: np.ndarray = field(init=False)
    world_to_robot_base: np.ndarray = field(init=False)
    world_to_external_camera: np.ndarray = field(init=False)
    external_camera_to_world: np.ndarray = field(init=False)
    robot_base_to_world: np.ndarray = field(init=False)
    unity_to_world_fit_matrix: np.ndarray = field(
        default_factory=lambda: np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
    )
    unity_to_world_transform: np.ndarray = field(
        default_factory=lambda: np.array(
            [[0.0, 0.0, 1.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    )

    def __post_init__(self) -> None:
        calibration_dir = Path(self.calibration_dir)
        self.external_camera_to_robot_base = _load_matrix(
            calibration_dir / "external_camera_to_left_robot_base_transform.json"
        )
        self.left_wrist_camera_to_tcp = _load_matrix(calibration_dir / "left_wrist_camera_to_left_robot_tcp_transform.json")
        self.world_to_robot_base = _load_matrix(calibration_dir / "world_to_left_robot_base_transform.json")
        self.world_to_external_camera = np.linalg.inv(self.external_camera_to_robot_base) @ self.world_to_robot_base
        self.external_camera_to_world = np.linalg.inv(self.world_to_external_camera)
        self.robot_base_to_world = np.linalg.inv(self.world_to_robot_base)
        self.unity_fit_matrix = np.linalg.inv(self.robot_base_to_world[:3, :3]) @ self.unity_to_world_fit_matrix

    @classmethod
    def from_dir(cls, calibration_dir: Union[str, Path]) -> "SingleArmCalibration":
        return cls(calibration_dir=Path(calibration_dir))

    def unity_to_robot_pose(self, pose7d: np.ndarray) -> np.ndarray:
        pose7d = np.asarray(pose7d, dtype=np.float64).copy()
        if pose7d.shape != (7,):
            raise ValueError("pose7d must have shape (7,)")
        pose7d *= np.array([1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0], dtype=np.float64)
        rot_mat = pose7d_to_matrix(pose7d)[:3, :3]
        pos_vec = pose7d[:3]
        target_rot_mat = self.unity_fit_matrix @ rot_mat
        target_pos_vec = self.unity_fit_matrix @ pos_vec
        target_quat = _matrix_to_quat_wxyz(target_rot_mat)
        return np.concatenate([target_pos_vec, target_quat])

    def robot_to_unity_pose(self, pose7d: np.ndarray) -> np.ndarray:
        pose_in_robot_frame = pose7d_to_matrix(pose7d)
        pose_in_world_frame = self.robot_base_to_world @ pose_in_robot_frame
        pose_in_unity_frame = np.linalg.inv(self.unity_to_world_transform) @ pose_in_world_frame
        target_pos_vec = pose_in_unity_frame[:3, 3]
        rot_mat = pose_in_world_frame[:3, :3]
        target_rot_mat = np.linalg.inv(self.unity_to_world_fit_matrix) @ rot_mat
        target_quat = _matrix_to_quat_wxyz(target_rot_mat)
        target_quat *= np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
        return np.concatenate([target_pos_vec, target_quat])


def _matrix_to_quat_wxyz(matrix: np.ndarray) -> np.ndarray:
    # Uses a stable trace-based conversion without depending on scipy at import time.
    m = matrix
    trace = np.trace(m)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    else:
        if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    quat = np.array([w, x, y, z], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return quat
