from __future__ import annotations

from typing import Iterable

import numpy as np
from scipy.spatial.transform import Rotation


def as_pose7d(values: Iterable[float]) -> np.ndarray:
    pose = np.asarray(list(values), dtype=np.float64)
    if pose.shape != (7,):
        raise ValueError("pose must have shape (7,)")
    return pose


def as_pose6d(values: Iterable[float]) -> np.ndarray:
    pose = np.asarray(list(values), dtype=np.float64)
    if pose.shape != (6,):
        raise ValueError("pose must have shape (6,)")
    return pose


def wxyz_to_xyzw(quaternion_wxyz: Iterable[float]) -> np.ndarray:
    quat = np.asarray(list(quaternion_wxyz), dtype=np.float64)
    if quat.shape != (4,):
        raise ValueError("quaternion must have shape (4,)")
    return np.array([quat[1], quat[2], quat[3], quat[0]], dtype=np.float64)


def xyzw_to_wxyz(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    quat = np.asarray(list(quaternion_xyzw), dtype=np.float64)
    if quat.shape != (4,):
        raise ValueError("quaternion must have shape (4,)")
    return np.array([quat[3], quat[0], quat[1], quat[2]], dtype=np.float64)


def pose7d_to_pose6d(pose7d: Iterable[float]) -> np.ndarray:
    pose = as_pose7d(pose7d)
    rotation = Rotation.from_quat(wxyz_to_xyzw(pose[3:]))
    return np.concatenate([pose[:3], rotation.as_rotvec()])


def pose6d_to_pose7d(pose6d: Iterable[float]) -> np.ndarray:
    pose = as_pose6d(pose6d)
    quaternion_xyzw = Rotation.from_rotvec(pose[3:]).as_quat()
    return np.concatenate([pose[:3], xyzw_to_wxyz(quaternion_xyzw)])


def pose7d_to_matrix(pose7d: Iterable[float]) -> np.ndarray:
    pose = as_pose7d(pose7d)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(wxyz_to_xyzw(pose[3:])).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def pose6d_to_matrix(pose6d: Iterable[float]) -> np.ndarray:
    pose = as_pose6d(pose6d)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_rotvec(pose[3:]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def matrix_to_pose6d(matrix: np.ndarray) -> np.ndarray:
    if matrix.shape != (4, 4):
        raise ValueError("matrix must have shape (4, 4)")
    rotation = Rotation.from_matrix(matrix[:3, :3])
    return np.concatenate([matrix[:3, 3], rotation.as_rotvec()])


def matrix_to_pose7d(matrix: np.ndarray) -> np.ndarray:
    pose6d = matrix_to_pose6d(matrix)
    return pose6d_to_pose7d(pose6d)


def pose6d_to_pose9d(pose6d: Iterable[float]) -> np.ndarray:
    matrix = pose6d_to_matrix(pose6d)
    rot_6d = matrix[:3, :2].T.reshape(-1)
    return np.concatenate([matrix[:3, 3], rot_6d])


def ortho6d_to_rotation_matrix(ortho6d: np.ndarray) -> np.ndarray:
    if ortho6d.ndim != 2 or ortho6d.shape[1] != 6:
        raise ValueError("ortho6d must have shape (N, 6)")
    x_raw = ortho6d[:, 0:3]
    y_raw = ortho6d[:, 3:6]
    x = normalize_vectors(x_raw)
    z = normalize_vectors(np.cross(x, y_raw))
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=-1)


def normalize_vectors(vectors: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    norms = np.maximum(norms, 1e-8)
    return vectors / norms

