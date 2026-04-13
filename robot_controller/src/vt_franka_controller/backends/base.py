from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence

import numpy as np

from vt_franka_shared.models import ControllerState


class FrankaBackend(ABC):
    name: str

    @abstractmethod
    def get_tcp_pose(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def get_controller_state(self, control_frequency_hz: float) -> ControllerState:
        raise NotImplementedError

    @abstractmethod
    def start_cartesian_impedance(self, stiffness: Sequence[float], damping: Sequence[float]) -> None:
        raise NotImplementedError

    @abstractmethod
    def update_desired_tcp(self, target_pose6d: np.ndarray) -> None:
        raise NotImplementedError

    @abstractmethod
    def move_gripper(self, width: float, velocity: float, force_limit: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def grasp(self, velocity: float, force_limit: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop_gripper(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def go_home(self, joint_positions: Sequence[float], duration_sec: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def shutdown(self) -> None:
        raise NotImplementedError

